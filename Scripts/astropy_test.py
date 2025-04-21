from astropy.coordinates import EarthLocation, AltAz, SkyCoord
from astropy.time import Time
from datetime import datetime, timedelta
from astropy.coordinates import get_body
import astropy.units as u

# Define the observer's location (latitude, longitude, and elevation)
location = EarthLocation(lat=32.9167 * u.deg, lon=-117.1436 * u.deg, height=100 * u.m)  # Mira Mesa, San Diego

# Get user input for observation time and RA/Dec values
# Calculate today's date and 6 PM PST (convert to UTC)
now = datetime.utcnow()
today_6pm_pst = datetime(now.year, now.month, now.day, 18, 0, 0) - timedelta(hours=8)  # PST is UTC-8
user_time = (today_6pm_pst + timedelta(hours=8)).strftime("%Y-%m-%d %H:%M:%S")  # Convert back to UTC string

# Get Jupiter's RA/Dec coordinates for the current time
jupiter_coord = get_body('jupiter', Time(user_time))
ra = jupiter_coord.ra.deg  # Right Ascension in degrees
dec = jupiter_coord.dec.deg  # Declination in degrees

# Convert the user-provided time to an astropy Time object
observation_time = Time.now()

# Create a SkyCoord object for the user-provided RA/Dec
sky_coord = SkyCoord(ra=ra * u.deg, dec=dec * u.deg, frame='icrs', obstime = user_time)

# Loop to calculate Alt/Az coordinates every second for a period of 10 seconds
for i in range(10):
    # Calculate the time i seconds into the future
    future_time = observation_time + i * u.s

    # Create an AltAz frame for the observer's location and future time
    altaz_frame = AltAz(obstime=future_time, location=location)

    # Convert the RA/Dec coordinates to Alt/Az at the future time
    altaz_coord = sky_coord.transform_to(altaz_frame)

    # Print the Alt/Az coordinates
    print(f"Time: {future_time.iso}")
    print(f"Altitude: {altaz_coord.alt}")
    print(f"Azimuth: {altaz_coord.a }")
