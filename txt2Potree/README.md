# txt2Potree
Program to convert point clouds in text files to Potree format.

### Requiriments:
* Numpy
* Pandas
* laspy

### Instruction:
1) setup PotreeConverter from https://github.com/potree/PotreeConverter
2) setup Potree from https://github.com/potree/potree
3) set variables in config.ini (one level up)
4) use with:
```
python txt2Potree.py -i input_file
```
### Input parameters:
```-h```; ```--help``` - display help

```-i```; ```--input``` - specify input file

```-s``` ; ```--seperator``` - specify how columns are seperated (default = ',') 

```-c```; ```--classification_column``` - specify which is the classification column (not index) (default = 4)
