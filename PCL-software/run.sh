#!bin/bash.sh

script_dir=$PWD

input_csv=$1 #get command line argument
input_csv=$(realpath "$input_csv")

points_per_cloud=$2

PCD_dir=$3
PCD_dir=$(realpath "$PCD_dir")





cd csv2pcd
#check if build folder exists
if ! test -d "build"; then #csv2pcd needs to be compiled
	mkdir -p build
	cd build
	cmake ..
	make
fi

cd build

#convert *.csv to *.pcd files
./csv2pcd $input_csv $points_per_cloud $PCD_dir

cd $script_dir/renderPCD

#check if build folder exists
if ! test -d "build"; then #renderPCD needs to be compiled
	mkdir -p build
	cd build
	cmake ..
	make
fi

cd build

#render pcd files
./renderPCD $PCD_dir
