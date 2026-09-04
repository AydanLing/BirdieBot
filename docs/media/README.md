# Demo media

The README shows a screen recording uploaded as a GitHub asset, linked at
the top of it. Re-record with the recipe below and replace that link.

## Recording it

```bash
ros2 launch grab_sequence grasp_trial.launch.py headless:=False shuttles:=4
```

Bringup takes about 3 minutes; the first pick starts after that. One pick runs
80-100 s and the full four take roughly 6 minutes.

What is worth capturing, in order of how well it reads:

1. **One complete pick**, from the robot turning toward a shuttlecock to the
   arm dropping it in the hopper. About 90 s, and it shows every stage: drive,
   visual alignment, grasp, the staged carry, release.
2. The **arm's carry** on its own, tightly cropped. Rise, tilt, traverse,
   release, in about 15 s. This is the part that looks deliberate rather than
   lucky.
3. A **time-lapse of all four**, sped up 8-10x, showing the court emptying.

For a README a 10-15 s loop is about right; anything longer and people scroll
past. Option 2 loops well.

## Turning a recording into a gif

```bash
ffmpeg -i capture.mp4 -vf "fps=12,scale=720:-1:flags=lanczos" -loop 0 demo.gif
gifsicle -O3 --lossy=60 demo.gif -o demo.gif    # if it is over ~5 MB
```

GitHub will not render a gif much over 10 MB in a README, and 720 px wide is
plenty for the page. If the file will not come down, an mp4 uploaded directly
into the README via the GitHub web editor plays inline too, and looks better.
