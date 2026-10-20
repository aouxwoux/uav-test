# Running and presenting the demo

From the repository root, on Ubuntu or Ubuntu in WSL:

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake libopencv-dev
bash run_demo.sh
```

The script builds `run_demo`, a C++ executable. It generates the imagery, calls the same `uavloc::Localizer` used by the batch CLI, writes `results.csv`, checks the results against synthetic truth, and creates `report.html` with predicted footprints. Matching does not read the truth file. There are no remote APIs, pretrained weights, or Python dependencies.

Open the printed `report.html` path directly in a browser. All images and scripts are local, so a web server is unnecessary. On WSL, a path starting `/mnt/c/` corresponds to `C:\` in Windows Explorer. Open the file from there.

Each default run gets a new output folder under `build/`. You can also choose a new folder:

```bash
bash run_demo.sh demo-output
```

An existing output folder is never overwritten. After the first build, you can invoke the C++ executable directly:

```bash
./build/run_demo another-demo
```

The browser report displays results from that run; switching tabs does not rerun inference. To rerun the matcher, execute the command again. For your own images, use the batch CLI and the [documented manifest format](approach.md#using-your-own-images).

## What to show

1. **Daylight:** input image on the left, predicted footprint and centre on the map at right.
2. **IR proxy:** the gradient branch matches an inverted, blurred image. Explicitly call this a proxy.
3. **Blank image:** insufficient texture, so no coordinates.
4. **Unrelated image:** there are features, but no verified match, so no coordinates.

The small synthetic errors are on views generated from the reference itself. They are not measured field accuracy. Real thermal matching and Jetson performance are still untested.

## Video

[demo.mp4](demo.mp4) is a 46-second silent walkthrough with captions. It combines captured process output and browser screenshots from a successful local C++ run. Build output is condensed and local paths are shortened. The video includes all four results and the synthetic-data limitation.

## Docker alternative

```bash
docker build -t uav-localizer .
mkdir -p demo-output
docker run --rm --network none \
  -v "$PWD/demo-output:/output" \
  --entrypoint /opt/uavloc/build/run_demo \
  uav-localizer /output/run
```

Open `demo-output/run/report.html` on the host. Choose a new folder for a second run. This is a portable CPU container starter; local validation used WSL, not a running Docker daemon or Jetson.
