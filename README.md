# AVC_2015_GizmoBot1
<p>Repository for AVC code.</p>

<p>This is the codebase for The Gizmo Dojo's 2015 Sparkfun AVC entry. Documentation will be sketchy,
but the basics should be here.</p>

<p>I've written the beginnings of a library for our very specific hardware setup. This is designed to prevent
damage to the control board.</p>

<p>The following commands can be used to move the car:</p>
<code>
init_movement();&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;// Please call this at the beginning of setup().<br />
move_forward(time_in_ms);&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;// This moves forward &lt;time_in_ms&gt; and coasts.<br />
move_forward_stop(time_in_ms);&nbsp;&nbsp;// This moves forward &lt;time_in_ms&gt; and stops.<br />
move_reverse(time_in_ms);&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;// This moves backwards &lt;time_in_ms&gt; and coasts.<br />
move_reverse_stop(time_in_ms);&nbsp;&nbsp;// This moves backwards &lt;time_in_ms&gt; and stops.<br />
wheels_left();&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;// This turns the wheels to the left position.<br />
wheels_right();&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;// This turns the wheels to the right position.<br />
wheels_center();&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;// This turns the wheels to the center (or straight) position.<br />
</code>
<br />&nbsp;<br />
More commands may be added if they are deemed useful.
