# uav visual localizer

trying to find where a drone image belongs on a stored map when gps isn't available. built with c++17 and opencv. runs offline.
we narrow the search using the approximate location, match visual features, then verify the geometry. enough consistent matches give us the image centre on the map. otherwise we return no match. the point is to avoid giving a confident wrong location.

## the idea

start with the rough location, look for matching corners and edges, then check if they agree with one perspective transform. if they do, project the image centre onto the map and return lat/lon. if the evidence is weak, return `no_match`.

i kept it basic: ORB for features, RANSAC to check the geometry, and a gradient branch to try matching structure when brightness changes. no model training or api keys.

## run the demo

ubuntu, or ubuntu in wsl on windows:

```bash
sudo apt-get update
sudo apt-get install -y git build-essential cmake libopencv-dev
git clone https://github.com/aouxwoux/ideaforge-uav-localizer.git
cd ideaforge-uav-localizer
bash run_demo.sh
```

it builds the c++ code, generates four images, runs the matcher and prints the path to `report.html`. open that file in your browser. switch between cases to see the input, predicted footprint, coordinates and result. the csv is there too. each run uses a new folder.

- daylight + inverted image: should match
- blank + unrelated image: should return no match

the report comes from the actual c++ run. it doesn't need a server or internet.

## video

https://github.com/user-attachments/assets/2bf7b374-c319-47df-90ae-bdccae0736c2

[download the video](docs/demo.mp4)




## where it's at

working prototype with synthetic demo data. the inverted image is an IR proxy, not real thermal footage. real EO/IR accuracy and jetson performance still need testing. confidence is a rough score, not a probability.

[demo notes](docs/demo.md) · [full approach + own-image setup](docs/approach.md) · [test results](docs/validation.md) · MIT
