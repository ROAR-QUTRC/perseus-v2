---
hide-toc: true
sd_hide_title: true
---

# Perseus-Lite

```{raw} html
<section class="pl-hero">
  <div class="pl-hero__glow" aria-hidden="true"></div>
  <div class="pl-hero__grid" aria-hidden="true"></div>
  <div class="pl-hero__inner">
    <div class="pl-hero__copy">
      <span class="pl-hero__eyebrow">ROS 2 Jazzy &middot; 4-Wheel Skid-Steer</span>
      <h1 class="pl-hero__title">
        <span class="pl-hero__title-line">PERSEUS</span><span class="pl-hero__title-dash">&mdash;</span><span class="pl-hero__title-line pl-hero__title-line--accent">LITE</span>
      </h1>
      <p class="pl-hero__lede">
        A 50%-scale exploration rover. Feetech ST3215 servos over a single serial cable,
        rocker suspension, SLAM, autonomy, vision, and a manipulator arm &mdash; all driven
        by a single Pixi-managed workspace.
      </p>
      <div class="pl-hero__ctas">
        <a class="pl-btn pl-btn--primary" href="home/perseus-lite-operation/basic-operation.html">
          <span class="pl-btn__icon">&#9654;</span>
          <span>Drive it now</span>
        </a>
        <a class="pl-btn pl-btn--ghost" href="home/getting-started.html">
          <span>First-time setup</span>
          <span class="pl-btn__arrow">&rarr;</span>
        </a>
      </div>
      <dl class="pl-hero__stats">
        <div><dt>Wheels</dt><dd>4&times; ST3215</dd></div>
        <div><dt>Bus</dt><dd>USB Serial</dd></div>
        <div><dt>Distro</dt><dd>ROS 2 Jazzy</dd></div>
        <div><dt>Toolchain</dt><dd>Pixi / RoboStack</dd></div>
      </dl>
    </div>
    <div class="pl-hero__art">
      <div class="pl-hero__orbit pl-hero__orbit--1"></div>
      <div class="pl-hero__orbit pl-hero__orbit--2"></div>
      <div class="pl-hero__orbit pl-hero__orbit--3"></div>
      <img src="_static/Logo-Complex.png" alt="Perseus-Lite emblem" class="pl-hero__logo" />
    </div>
  </div>
</section>

<section class="pl-section">
  <header class="pl-section__head">
    <span class="pl-section__eyebrow">Capabilities</span>
    <h2 class="pl-section__title">What the rover can do</h2>
    <p class="pl-section__lede">Lite is intentionally small &mdash; not feature-poor. Every system below ships in this repo and is reachable from <code>pixi shell</code>.</p>
  </header>
  <div class="pl-cards">
    <article class="pl-card">
      <div class="pl-card__icon" aria-hidden="true">&#9881;</div>
      <h3>Skid-Steer Drive</h3>
      <p>Four-wheel chassis on a rocker suspension. <code>diff_drive_controller</code> turns <code>TwistStamped</code> commands into per-wheel velocities.</p>
      <span class="pl-card__tag">perseus_lite_hardware</span>
    </article>
    <article class="pl-card">
      <div class="pl-card__icon" aria-hidden="true">&#9881;</div>
      <h3>ST3215 Servos</h3>
      <p>Four Feetech ST3215 servos on a single USB serial bus (IDs 1&ndash;4). No CAN, no VESC, no custom PCB &mdash; just plug and drive.</p>
      <span class="pl-card__tag">/dev/ttyACM0</span>
    </article>
    <article class="pl-card">
      <div class="pl-card__icon" aria-hidden="true">&#9737;</div>
      <h3>SLAM &amp; Nav2</h3>
      <p>RPLidar feeds <code>slam_toolbox</code> for live mapping; Nav2 plans and executes. Behavior trees orchestrate higher-level autonomy.</p>
      <span class="pl-card__tag">autonomy</span>
    </article>
    <article class="pl-card">
      <div class="pl-card__icon" aria-hidden="true">&#128065;</div>
      <h3>Computer Vision</h3>
      <p>ONNX-based detectors for ArUco markers and target cubes, ready to be wired into your own perception or autonomy stack.</p>
      <span class="pl-card__tag">perseus_vision</span>
    </article>
    <article class="pl-card">
      <div class="pl-card__icon" aria-hidden="true">&#9999;</div>
      <h3>Manipulator Arm</h3>
      <p>Standalone C++ teleop drives a Feetech-servo arm directly over serial &mdash; no ROS bridge required for quick demos.</p>
      <span class="pl-card__tag">arm-teleop-direct</span>
    </article>
    <article class="pl-card">
      <div class="pl-card__icon" aria-hidden="true">&#9881;</div>
      <h3>Gazebo Simulation</h3>
      <p>A full sim of the lite chassis, controllers, twist_mux, RViz and EKF &mdash; the same launch graph as the real robot.</p>
      <span class="pl-card__tag">perseus_lite_simulation</span>
    </article>
  </div>
</section>

<section class="pl-section pl-section--quickstart">
  <header class="pl-section__head">
    <span class="pl-section__eyebrow">Quick start</span>
    <h2 class="pl-section__title">Teleoperate in two terminals</h2>
    <p class="pl-section__lede">Power the rover, plug the USB cable into your laptop, then open two shells inside <code>~/perseus-lite</code>. Everything below runs through <code>pixi</code> &mdash; no system packages required.</p>
  </header>
  <div class="pl-terminals">
    <div class="pl-term">
      <div class="pl-term__bar">
        <span class="pl-term__dot pl-term__dot--r"></span>
        <span class="pl-term__dot pl-term__dot--y"></span>
        <span class="pl-term__dot pl-term__dot--g"></span>
        <span class="pl-term__title">terminal 1 &middot; robot bringup</span>
      </div>
      <pre class="pl-term__body"><code><span class="pl-tok-c"># Drop into the Pixi-managed dev shell</span>
<span class="pl-tok-p">$</span> pixi shell
<span class="pl-tok-c"># Launch ros2_control, drivers, and twist_mux</span>
<span class="pl-tok-p">$</span> ros2 launch perseus_lite perseus_lite.launch.py \
      cmd_vel_topic:=/joy_vel</code></pre>
    </div>
    <div class="pl-term">
      <div class="pl-term__bar">
        <span class="pl-term__dot pl-term__dot--r"></span>
        <span class="pl-term__dot pl-term__dot--y"></span>
        <span class="pl-term__dot pl-term__dot--g"></span>
        <span class="pl-term__title">terminal 2 &middot; controller input</span>
      </div>
      <pre class="pl-term__body"><code><span class="pl-tok-c"># Xbox / generic gamepad teleop</span>
<span class="pl-tok-p">$</span> ros2 run input_devices generic_controller

<span class="pl-tok-c"># &mdash; or, if you prefer keyboard &mdash;</span>
<span class="pl-tok-p">$</span> ros2 run teleop_twist_keyboard teleop_twist_keyboard \
      --ros-args -p stamped:=true -r cmd_vel:=key_vel</code></pre>
    </div>
  </div>
  <div class="pl-quickstart-foot">
    <a class="pl-btn pl-btn--ghost" href="home/perseus-lite-operation/basic-operation.html">
      <span>Full operation guide</span>
      <span class="pl-btn__arrow">&rarr;</span>
    </a>
    <a class="pl-btn pl-btn--ghost" href="home/getting-started.html">
      <span>Dev environment setup</span>
      <span class="pl-btn__arrow">&rarr;</span>
    </a>
  </div>
</section>

<section class="pl-section pl-section--paths">
  <header class="pl-section__head">
    <span class="pl-section__eyebrow">Where to next</span>
    <h2 class="pl-section__title">Pick your path</h2>
  </header>
  <div class="pl-paths">
    <a class="pl-path" href="home/getting-started.html">
      <h3>I&rsquo;m new here</h3>
      <p>Install Pixi, clone the repo, and get a working dev shell on a fresh laptop.</p>
      <span class="pl-path__arrow">&rarr;</span>
    </a>
    <a class="pl-path" href="systems-index.html">
      <h3>I want to understand the systems</h3>
      <p>Hardware, software and autonomy &mdash; how the lite rover fits together.</p>
      <span class="pl-path__arrow">&rarr;</span>
    </a>
    <a class="pl-path" href="generated/exhale/index.html">
      <h3>Show me the code</h3>
      <p>Auto-generated C++ API reference for every public class and function in the workspace.</p>
      <span class="pl-path__arrow">&rarr;</span>
    </a>
  </div>
</section>
```

% note: elements in this toctree are manually ordered rather than just using a glob

```{toctree}
:maxdepth: 1
:hidden:
:glob:
self
home/getting-started
home/*
systems-index
standards-index
development-index
maintenance-index
tutorials-index
generated/exhale/index
```
