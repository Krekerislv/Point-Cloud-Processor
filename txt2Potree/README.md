# txt2Potree
Program to convert text files to Potree format

### Requiriments:
* Numpy
* Pandas
* laspy

### Instruction:
1) setup PotreeConverter from https://github.com/potree/PotreeConverter
2) setup Potree from https://github.com/potree/potree
3) set variables in config.ini
4) use with:
```
python csv2Potree.py -i input_file
```
### Flags:
```-h```; ```--help``` - display help

```-i```; ```--input``` - specify input file

```-s``` ; ```--seperator``` - specify how data is seperated (default = ',') 

```-c```; ```--classification``` - specify which is the classification column (default = 4)
