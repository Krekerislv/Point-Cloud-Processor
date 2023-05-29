# Cloud parameters

This Python program parses point cloud in a text file to determine total point count, noise point count, non-noise point count, cluster count and noise point classification accuracy score.

This program assumes that noise is  marked with label 255.

Use with:
```
python cloud_parameters.py -i "path/to/file.csv"
```

## Input parameters
* ```-h```; ```--help``` - Prints help message;

* ```-i```; ```--input``` - input point cloud text file;
* ```-s```; ```--seperator``` - specify how the columns are seperated (default = ',');
* ```-c```; ```--classification_column``` - specify which column (not index) contains classification labels (default = 4)
* ```-p```; ```--print``` - add this flag to print the output to console
* ```-gt```; ```--ground_truth``` - specify ground truth file in order to calculate noise classification accuracy score