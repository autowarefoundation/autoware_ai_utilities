# Lanelet Aisan Converter

This package contains tools to convert between Lanelet2 OSM and Aisan Vector Map.

## lanelet2aisan

This node converts Lanelet2 OSM file into Aisan Vector Map csv files.

### Usage

#### Command

`rosrun lanelet_aisan_converter lanelet2aisan _map_file:=<path to lanelet map> _origin_lat:=<latitude of origin for XYZ projection> _origin_lon:=<longitude of origin for XYZ projection> _save_dir:=<optional: path to save dir>`

For sample lanelet map:  
`` rosrun lanelet_aisan_converter lanelet2aisan _map_file:=`rospack find lanelet2_maps`/res/mapping_example.osm _origin_lat:=49.00331750371 _origin_lon:=8.42403027399 _save_dir:=./aisan_vector_map ``

#### Parameters

- ~map_file (string, required)  
  Lanelet2 OSM file

- ~save_dir (string, default: "./")  
  directory to save converted map

- ~origin_lat (double, required)  
  latitude origin of the map.

- ~origin_lon (double, required)  
  longitude origin of the map.

### Limitations

This tool doesn't support 100% conversion and assumes some manual adjustings using Vector Map Builder.  
Please see `create_feature.cpp` for more details about what attributes are not supported.  
When a certain attribute is not supported, there is a comment like this.

```cpp
road_sign.linkid = 0;  // Not supported
```
