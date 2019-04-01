if {[info proc tkgof] == ""} {rename tk_getOpenFile tkgof}
proc tk_getOpenFile {args} {
  # location on screen:
  set xoff 0
  set yoff 0
  # get width and height for max screen 
  set maxsize [wm maxsize .]
  set width   [lindex $maxsize 0]
  set height  [lindex $maxsize 1]
  # alternately, just set width, height, and offsets explicitly:
  #  set width  800
  #  set height 600
  #  set xoff   100
  #  set yoff   100
  after 0 [list wm geometry .__tk_filedialog ${width}x${height}+${xoff}+${yoff}]
  return [eval tkgof $args]
}
