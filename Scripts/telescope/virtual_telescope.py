import numpy as np

from .physical_telescope import PhysicalTelescope, DegMinSec, Axes
from uart_handler import UARTTxManager

from astropy.coordinates import EarthLocation, AltAz, SkyCoord, Angle
from astropy.time import Time
from datetime import datetime, timedelta
from astropy.coordinates import get_body
import threading
import astropy.units as u
import time


class VirtualTelescope:
    def __init__(self, tx_mgr: UARTTxManager, lat = 32.9167, long = -117.1436, height = 100):
        self.telescope = PhysicalTelescope(tx_mgr)

        self.lat = lat
        self.long = long
        self.height = 100

        self._loc = EarthLocation(lat=self.lat * u.deg, lon=self.long * u.deg, height=self.height * u.m)
        self._tracking = False
        self._tracking_thread = None

    def physical(self) -> PhysicalTelescope:
        return self.telescope

    def _to_deg_min_sec(self, coord: SkyCoord, observation_time):
        # Create a SkyCoord object for the user-provided RA/Dec
        altaz_frame = AltAz(obstime=observation_time, location=self._loc)
        altaz_coord = coord.transform_to(altaz_frame)

        pitch_deg, pitch_min, pitch_sec = Angle((90 * u.deg - altaz_coord.alt)).dms

        azimuth = altaz_coord.az
        if(azimuth > 180 * u.deg):
            azimuth = azimuth - 360 * u.deg
            
        yaw_sign, yaw_deg, yaw_min, yaw_sec = Angle(azimuth).signed_dms

        return (DegMinSec(yaw_sign * yaw_deg, yaw_sign * yaw_min, yaw_sign * yaw_sec), DegMinSec(0, 0, 0), DegMinSec(-pitch_deg, -pitch_min, -pitch_sec))

    def set_location(self, lat, long, height):
        self.lat = lat
        self.long = long
        self.height = height

        self._loc = EarthLocation(lat=self.lat * u.deg, lon=self.long * u.deg, height=self.height * u.m)

    def track_planet(self, name):
        obstime = Time.now() + 90 * u.s
        planet = get_body(name, obstime)
        yaw, roll, pitch = self._to_deg_min_sec(planet, obstime)

        self.telescope.goto(yaw, roll, pitch)

        def tracking_thread():
            
            """
                Wait for this to complete
            """
            while(self.telescope.is_processing()):
                pass

            self.telescope.start_tracking()

            while self._tracking:
                # Calculate the time i seconds into the future
                future_time = Time.now() + 1 * u.s
                yaw, roll, pitch = self._to_deg_min_sec(planet, future_time)
                self.telescope.track(yaw, roll, pitch)
                time.sleep(0.8)

            self.telescope.stop_tracking()

        self._tracking = True
        self._tracking_thread = threading.Thread(target=tracking_thread, daemon=True)
        self._tracking_thread.start()

    def stop_tracking(self):
        self._tracking = False
        self._tracking_thread.join()

    def is_tracking(self):
        return self._tracking
