
import os
import sys
import calendar
import requests
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from datetime import date, timedelta
from pathlib import Path
from matplotlib.backends.backend_pdf import PdfPages


# ============================================================
# CONFIGURATION
# ============================================================

API_KEY = os.getenv(
    "WU_API_KEY",
    "Left blank ask for API key"
)

STATION_ID = "IPOTCH10"

API_URL = "https://api.weather.com/v2/pws/history/hourly"

OUTPUT_DIR = Path("wind_rose_output")

# Wind speed conversion
# Weather Underground metric wind speeds are in km/h.
API_WIND_SPEED_UNIT = "km/h"

# Wind rose configuration
DIRECTION_SECTORS = 16

SPEED_BINS_MS = [
    0, 1, 2, 4, 6, 8, 10, np.inf
]

SPEED_LABELS = [
    "0–1 m/s",
    "1–2 m/s",
    "2–4 m/s",
    "4–6 m/s",
    "6–8 m/s",
    "8–10 m/s",
    "10+ m/s"
]

CALM_THRESHOLD = 0.5


# ============================================================
# FETCH WEATHER UNDERGROUND DATA
# ============================================================

def parse_observations(payload):

    observations = payload.get("observations", [])

    rows = []

    for obs in observations:

        metric = obs.get("metric") or {}

        # Support common API field-name variants
        speed = metric.get(
            "windSpeed",
            metric.get("windspeedAvg")
        )

        direction = obs.get(
            "winddir",
            obs.get("winddirAvg")
        )

        timestamp = (
            obs.get("obsTimeLocal")
            or obs.get("obsTimeUtc")
        )

        if timestamp is None:
            continue

        if speed is None or direction is None:
            continue

        try:

            speed = float(speed)
            direction = float(direction)

        except (TypeError, ValueError):
            continue

        if not np.isfinite(speed):
            continue

        if not np.isfinite(direction):
            continue

        if speed < 0:
            continue

        if not 0 <= direction <= 360:
            continue

        # Convert wind speed to m/s

        if API_WIND_SPEED_UNIT == "km/h":

            speed = speed / 3.6

        elif API_WIND_SPEED_UNIT == "mph":

            speed = speed * 0.44704

        elif API_WIND_SPEED_UNIT != "m/s":

            raise ValueError(
                "Invalid wind speed unit configuration."
            )

        # Preserve station-local timestamps when available
        timestamp = pd.to_datetime(
            timestamp,
            errors="coerce"
        )

        if pd.isna(timestamp):
            continue

        rows.append({
            "time": timestamp,
            "winddir": direction % 360,
            "windspeed_ms": speed
        })

    return pd.DataFrame(
        rows,
        columns=[
            "time",
            "winddir",
            "windspeed_ms"
        ]
    )


# ============================================================
# FETCH DATE RANGE
# ============================================================

def fetch_range(start_day, end_day):

    all_frames = []

    chunk_start = start_day

    while chunk_start <= end_day:

        # Maximum 31-day request
        chunk_end = min(
            chunk_start + timedelta(days=30),
            end_day
        )

        params = {
            "stationId": STATION_ID,
            "format": "json",
            "units": "m",
            "numericPrecision": "decimal",
            "startDate": chunk_start.strftime("%Y%m%d"),
            "endDate": chunk_end.strftime("%Y%m%d"),
            "apiKey": API_KEY
        }

        print(
            f"Fetching {chunk_start} "
            f"to {chunk_end} ..."
        )

        response = requests.get(
            API_URL,
            params=params,
            timeout=60
        )

        if response.status_code >= 400:

            print(
                f"\nWeather Underground error: "
                f"{response.status_code}"
            )

            print(response.text[:1000])

            response.raise_for_status()

        payload = response.json()

        if payload.get("errors") or payload.get("error"):

            raise RuntimeError(
                f"API error: {payload}"
            )

        frame = parse_observations(payload)

        if not frame.empty:

            all_frames.append(frame)

        chunk_start = chunk_end + timedelta(days=1)

    if not all_frames:

        return pd.DataFrame({
            "time": pd.Series(dtype="datetime64[ns]"),
            "winddir": pd.Series(dtype="float64"),
            "windspeed_ms": pd.Series(dtype="float64")
        })

    result = pd.concat(
        all_frames,
        ignore_index=True
    )

    # IMPORTANT: Ensure time is datetime
    result["time"] = pd.to_datetime(
        result["time"],
        errors="coerce"
    )

    result = result.dropna(
        subset=[
            "time",
            "winddir",
            "windspeed_ms"
        ]
    )

    result = result.drop_duplicates(
        subset=["time"]
    )

    result = result.sort_values(
        "time"
    )

    return result.reset_index(drop=True)


# ============================================================
# WIND ROSE GENERATION
# ============================================================

def draw_wind_rose(data, title, output_path):

    data = data.copy()

    data = data.dropna(
        subset=[
            "winddir",
            "windspeed_ms"
        ]
    )

    if data.empty:

        print(f"No data available for {title}")

        return None

    # Separate calm observations

    calm_data = data[
        data["windspeed_ms"] < CALM_THRESHOLD
    ]

    directional = data[
        data["windspeed_ms"] >= CALM_THRESHOLD
    ]

    calm_percentage = (
        len(calm_data) / len(data)
    ) * 100

    # Create polar plot

    fig = plt.figure(figsize=(9, 8))

    ax = fig.add_subplot(
        111,
        polar=True
    )

    ax.set_theta_zero_location("N")

    ax.set_theta_direction(-1)

    if directional.empty:

        ax.text(
            0.5,
            0.5,
            "No directional wind observations",
            transform=ax.transAxes,
            ha="center",
            va="center"
        )

    else:

        sector_width = 360 / DIRECTION_SECTORS

        sector_centers_deg = np.arange(
            DIRECTION_SECTORS
        ) * sector_width

        sector_centers_rad = np.deg2rad(
            sector_centers_deg
        )

        # Assign direction to 16 sectors

        sector_index = np.floor(
            (
                directional["winddir"].to_numpy()
                + sector_width / 2
            ) / sector_width
        ).astype(int) % DIRECTION_SECTORS

        # Assign wind speed to speed bands

        speed_index = pd.cut(
            directional["windspeed_ms"],
            bins=SPEED_BINS_MS,
            labels=False,
            right=False,
            include_lowest=True
        )

        valid = ~pd.isna(speed_index)

        sector_index = sector_index[valid]

        speed_index = np.asarray(
            speed_index[valid],
            dtype=int
        )

        # Frequency matrix

        counts = np.zeros(
            (
                len(SPEED_LABELS),
                DIRECTION_SECTORS
            )
        )

        for sector, speed_band in zip(
            sector_index,
            speed_index
        ):

            counts[speed_band, sector] += 1

        # Calculate percentage of ALL observations

        total_observations = len(data)

        bottom = np.zeros(
            DIRECTION_SECTORS
        )

        for band, label in enumerate(SPEED_LABELS):

            heights = (
                counts[band]
                / total_observations
            ) * 100

            ax.bar(
                sector_centers_rad,
                heights,
                width=np.deg2rad(
                    sector_width * 0.88
                ),
                bottom=bottom,
                align="center",
                edgecolor="white",
                linewidth=0.5,
                label=label
            )

            bottom += heights

        compass_labels = [
            "N", "NNE", "NE", "ENE",
            "E", "ESE", "SE", "SSE",
            "S", "SSW", "SW", "WSW",
            "W", "WNW", "NW", "NNW"
        ]

        ax.set_xticks(
            sector_centers_rad
        )

        ax.set_xticklabels(
            compass_labels
        )

        ax.set_ylabel(
            "Frequency (% of all observations)",
            labelpad=25
        )

        ax.legend(
            title="Wind speed",
            loc="upper left",
            bbox_to_anchor=(1.08, 1.08)
        )

    # Chart title

    ax.set_title(
        f"{title}\n"
        f"Observations: {len(data):,} | "
        f"Calm: {calm_percentage:.1f}%",
        pad=25
    )

    ax.grid(
        True,
        alpha=0.4
    )

    fig.tight_layout()

    # Save PNG

    fig.savefig(
        output_path,
        dpi=220,
        bbox_inches="tight"
    )

    print(
        f"Saved: {output_path}"
    )

    return fig


# ============================================================
# MAIN PROGRAM
# ============================================================

def main():

    if (
        not API_KEY
        or API_KEY == "PASTE_YOUR_WEATHER_UNDERGROUND_API_KEY_HERE"
    ):

        raise SystemExit(
            "Please enter your Weather Underground API key."
        )

    OUTPUT_DIR.mkdir(
        parents=True,
        exist_ok=True
    )

    today = date.today()

    yesterday = today - timedelta(days=1)

    year_start = date(
        today.year,
        1,
        1
    )

    print("\n===================================")
    print(" WEATHER UNDERGROUND WIND ANALYSIS")
    print("===================================\n")

    # --------------------------------------------------------
    # DOWNLOAD CURRENT YEAR DATA
    # --------------------------------------------------------

    year_data = fetch_range(
        year_start,
        yesterday
    )

    # Ensure datetime format before filtering

    year_data["time"] = pd.to_datetime(
        year_data["time"],
        errors="coerce"
    )

    year_data = year_data.dropna(
        subset=["time"]
    )

    year_data = year_data.sort_values(
        "time"
    ).reset_index(drop=True)

    if year_data.empty:

        raise RuntimeError(
            "No valid wind observations were returned."
        )

    # Save yearly CSV

    year_csv = OUTPUT_DIR / (
        f"wind_data_{today.year}.csv"
    )

    year_data.to_csv(
        year_csv,
        index=False
    )

    print(
        f"\nSaved observations: {year_csv}"
    )

    # --------------------------------------------------------
    # GENERATE FIVE DAILY WIND ROSES
    # --------------------------------------------------------

    charts = []

    for offset in range(1, 6):

        target_day = today - timedelta(
            days=offset
        )

        daily = year_data[
            year_data["time"].dt.date == target_day
        ].copy()

        print(
            f"\nDaily analysis: {target_day}"
        )

        daily_csv = OUTPUT_DIR / (
            f"wind_data_{target_day.isoformat()}.csv"
        )

        daily.to_csv(
            daily_csv,
            index=False
        )

        daily_png = OUTPUT_DIR / (
            f"wind_rose_{target_day.isoformat()}.png"
        )

        fig = draw_wind_rose(
            daily,
            f"Daily Wind Rose - "
            f"{target_day.strftime('%d %B %Y')}",
            daily_png
        )

        if fig is not None:

            charts.append(fig)

    # --------------------------------------------------------
    # GENERATE MONTHLY WIND ROSE
    # --------------------------------------------------------

    # --------------------------------------------------------
    # GENERATE LAST TWO COMPLETED/CURRENT MONTH WIND ROSES
    # --------------------------------------------------------

    for offset in range(2):

        # Calculate month relative to current month
        month_index = today.year * 12 + today.month - 1 - offset

        year = month_index // 12
        month = month_index % 12 + 1

        month_start = date(year, month, 1)

        last_day = calendar.monthrange(year, month)[1]

        month_end = date(year, month, last_day)

        # Exclude today and future dates
        month_end = min(month_end, yesterday)

        print(
            f"\nGenerating {calendar.month_name[month]} "
            f"{year} wind rose..."
        )

        monthly = fetch_range(
            month_start,
            month_end
        )

        monthly_png = OUTPUT_DIR / (
            f"wind_rose_month_{year}_{month:02d}.png"
        )

        monthly_csv = OUTPUT_DIR / (
            f"wind_data_month_{year}_{month:02d}.csv"
        )

        monthly.to_csv(
            monthly_csv,
            index=False
        )

        monthly_fig = draw_wind_rose(
            monthly,
            f"Monthly Wind Rose - "
            f"{calendar.month_name[month]} {year}",
            monthly_png
        )

        if monthly_fig is not None:
            charts.append(monthly_fig)

    # --------------------------------------------------------
    # GENERATE YEARLY WIND ROSE
    # --------------------------------------------------------

    yearly = year_data[
        year_data["time"].dt.date <= yesterday
    ].copy()

    yearly_png = OUTPUT_DIR / (
        f"wind_rose_year_{today.year}.png"
    )

    yearly_fig = draw_wind_rose(
        yearly,
        f"Year to date Wind Rose - {today.year}",
        yearly_png
    )

    if yearly_fig is not None:

        charts.append(yearly_fig)

    # --------------------------------------------------------
    # EXPORT ALL CHARTS TO ONE PDF
    # --------------------------------------------------------

    pdf_path = OUTPUT_DIR / (
        f"Wind_Roses_{today.year}.pdf"
    )

    with PdfPages(pdf_path) as pdf:

        for fig in charts:

            pdf.savefig(
                fig,
                bbox_inches="tight"
            )

    print(
        f"\nPDF saved: {pdf_path}"
    )

    # --------------------------------------------------------
    # DISPLAY RESULTS
    # --------------------------------------------------------

    print("\n===================================")
    print(" WIND ANALYSIS COMPLETE")
    print("===================================")

    print(
        f"Total observations: {len(year_data):,}"
    )

    print(
        f"Daily charts: 5 requested"
    )

    print(
        f"Monthly observations: {len(monthly):,}"
    )

    print(
        f"Yearly observations: {len(yearly):,}"
    )

    print(
        f"\nAll results saved in: {OUTPUT_DIR.resolve()}"
    )

    # Display each chart

    plt.show()

    # Close figures after display

    for fig in charts:

        plt.close(fig)


# ============================================================
# RUN
# ============================================================

if __name__ == "__main__":

    main()
