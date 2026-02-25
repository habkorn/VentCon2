"""
PlatformIO pre-build script: inject VENTCON_VERSION from the VERSION file.

Reads the project-root VERSION file and adds a -D build flag so that
Constants.h can use it without hard-coding the version string.
"""
Import("env")
import os

version_file = os.path.join(env["PROJECT_DIR"], "VERSION")
with open(version_file, "r") as f:
    version = f.read().strip()

# Append the define; Constants.h will compose the full string with __DATE__/__TIME__
env.Append(
    CPPDEFINES=[("VENTCON_VERSION_NUMBER", env.StringifyMacro(version))]
)
print(f"  ** inject_version.py: VENTCON_VERSION_NUMBER = {version}")
