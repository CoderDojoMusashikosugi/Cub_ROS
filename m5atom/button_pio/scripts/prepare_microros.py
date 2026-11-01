"""Adapt the pinned micro-ROS builder to PlatformIO 8's nested SCons flags.

Only the downloaded dependency is patched; our source and global packages are not.
"""
from pathlib import Path
import subprocess

Import("env")

script = (Path(env.subst("$PROJECT_LIBDEPS_DIR")) / env.subst("$PIOENV")
          / "micro_ros_platformio" / "extra_script.py")
if not script.exists():
    subprocess.run([env.subst("$PYTHONEXE"), "-m", "platformio", "pkg", "install",
                    "-e", env.subst("$PIOENV")], check=True)
if script.exists():
    original = script.read_text()
    source = original
    for flag in ("CFLAGS", "CXXFLAGS", "CCFLAGS"):
        source = source.replace("' '.join(env['%s'])" % flag,
                                'env.subst("$%s")' % flag)
    source = source.replace('env.subst("$CCFLAGS")',
                            'env.subst("$CCFLAGS $_CPPINCFLAGS")')
    source = source.replace('["clean_microros", "clean_libmicroros", "_idedata", "idedata"]',
                            '["clean_microros", "clean_libmicroros", "_idedata", "__idedata", "idedata"]')
    if source != original:
        script.write_text(source)
    builder = script.parent / "microros_utils" / "library_builder.py"
    original = builder.read_text()
    source = original
    # ESP-IDF's assert.h keeps assertions enabled according to sdkconfig even
    # when NDEBUG is set. Jazzy guards expected type hashes with !NDEBUG.
    release_flags = ("-DCMAKE_BUILD_TYPE=Release "
                     '-DCMAKE_C_FLAGS_RELEASE="-Os -UNDEBUG" '
                     '-DCMAKE_CXX_FLAGS_RELEASE="-Os -UNDEBUG"')
    for language in ("C", "CXX"):
        source = source.replace("-DCMAKE_%s_FLAGS_RELEASE='-Os -UNDEBUG'" % language,
                                '-DCMAKE_%s_FLAGS_RELEASE="-Os -UNDEBUG"' % language)
    if "-DCMAKE_C_FLAGS_RELEASE=" not in source:
        source = source.replace("-DCMAKE_BUILD_TYPE=Release", release_flags)
    if source != original:
        builder.write_text(source)
