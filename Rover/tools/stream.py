#!/usr/bin/env python3
"""
List available RealSense devices and their stream profiles.

Run:
	python Rover/tools/stream.py

This prints each device, its sensors, and the supported stream profiles
including resolution, format and framerate.
"""
import traceback

try:
	import pyrealsense2 as rs
except Exception as e:
	print("ERROR: failed importing pyrealsense2:", e)
	raise


def stream_name(stream_enum):
	try:
		return str(stream_enum)
	except Exception:
		return repr(stream_enum)


def fmt_name(fmt_enum):
	try:
		return str(fmt_enum)
	except Exception:
		return repr(fmt_enum)


def main():
	ctx = rs.context()
	devices = ctx.query_devices()
	if len(devices) == 0:
		print("No RealSense devices found")
		return

	print(f"Found {len(devices)} device(s):\n")
	for i, dev in enumerate(devices):
		try:
			serial = dev.get_info(rs.camera_info.serial_number)
		except Exception:
			serial = "<unknown>"
		try:
			name = dev.get_info(rs.camera_info.name)
		except Exception:
			name = "<unknown>"
		try:
			fw = dev.get_info(rs.camera_info.firmware_version)
		except Exception:
			fw = "<unknown>"

		print(f"Device {i}: serial={serial} name={name} fw={fw}")

		try:
			sensors = dev.query_sensors()
		except Exception:
			sensors = []

		for s_idx, sensor in enumerate(sensors):
			try:
				sname = sensor.get_info(rs.camera_info.name)
			except Exception:
				sname = f"sensor_{s_idx}"
			print(f"  Sensor {s_idx}: {sname}")

			try:
				profiles = sensor.get_stream_profiles()
			except Exception:
				profiles = []

			for p in profiles:
				try:
					st = p.stream_type()
				except Exception:
					st = "?"
				try:
					fm = p.format()
				except Exception:
					fm = "?"
				try:
					fps = p.fps()
				except Exception:
					fps = "?"

				line = f"    - stream={stream_name(st)} fmt={fmt_name(fm)} fps={fps}"
				# If video profile, show resolution
				try:
					if p.is_video_stream_profile():
						v = p.as_video_stream_profile()
						line += f" res={v.width}x{v.height}"
				except Exception:
					pass

				print(line)

		print("")


if __name__ == '__main__':
	try:
		main()
	except Exception:
		traceback.print_exc()
