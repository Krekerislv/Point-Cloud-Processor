# findGroundTruth.py
## A program that parses raw data and looks for a point (in given radius) in clean data set.

# Requiremenets
* numpy
* pandas
* scikit-learn

### Use with:
```
python findGroundTruth.py --raw path/to/raw/file.csv --clean path/to/clean/file.csv
```

## Input parameters
* ```-h```; ```--help``` - Prints help message;

* ```-dr```; ```--raw``` - raw data input;

* ```-dc```; ```--clean``` - clean data input;

* ```-r```; ```--radius``` - in what radius to look for clean data point;

* ```-sep```; ```--seperator``` - specify how the columns are seperated (default = ',');

* ```-c```; ```--cores``` - specify how many CPU cores to use in parallel (default = 4)

* ```-s```; ```--save``` - specify file where to save ground truth