# Point cloud cleaner
Program to clean point clouds with DBSCAN, save results in a text file and convert to Potree format to visualize results.

### Requiriments:
* Numpy
* Pandas
* laspy
* sklearn

### Instruction:
1) setup PotreeConverter from https://github.com/potree/PotreeConverter
2) setup Potree from https://github.com/potree/potree
3) set variables in config.ini (one level up)
4) use with:
```
python point_cloud_cleaner.py -i input_file
```
### Input parameters:
```-h```; ```--help``` - display help

```-i```; ```--input``` - specify input file(s). All files must be with same column count

```-n```; ```--name``` - Specify the name of point cloud (how it will display in Potree)

```-s``` ; ```--seperator``` - specify how columns are seperated (default = ',')

```--classification_column``` - specify which is the classification column (not index) (default = 4)

```--save``` - Specify file if you wish to save text file after clustering


#### Input parameters for denoising:
```-c```;```--classifyNoise``` - add this to classify noise (otherwise point cloud will just be converted to Potree format)

```--cores``` - specify how many CPU cores to use in parallel (-1 for all available, default = 4)

```--eps``` - neighnorhood distance for point (DBSCAN)

```--min_samples``` - minimum number of neighbouring points needed for point to be considered a core point (DBSCAN)
