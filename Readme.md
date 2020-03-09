# Useful functions for point cloud processing (implemented with pcl)

## How to use:

1. Compile
```
mkdir build
cd build
cmake ..
make 
```

2. Run
```
cd ..
# configure the script/run.sh file for editting the data path
# run the function
sh script/run_xxx.sh
```


## Notes:
The plane coefficients file should be set as follows.

```
a b c d
```

which meets the 3D plane equation: ax+by+cz+d=0