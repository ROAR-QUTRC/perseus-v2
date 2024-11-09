{
  colconWithExtensions,
  writeShellScriptBin,
  gnugrep,
}:
# pipe colcon output through grep to filter out the ament prefix path warnings,
# as well as setting PYTHONWARNINGS to silence easy_install and unknown distribution warnings
writeShellScriptBin "colcon" ''
  PYTHONWARNINGS="ignore:easy_install command is deprecated,ignore:Unknown distribution option:UserWarning" \
    exec ${colconWithExtensions}/bin/colcon "$@" 2> >(${gnugrep}/bin/grep -v "WARNING:colcon.colcon_ros.prefix_path.ament" >&2)
''
