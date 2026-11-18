# sfg_go2_rssi_costmap_plugin

This package contains the initial RSSI costmap layer for Nav2.

The layer keeps an internal `qos_cost_` grid at the master costmap's
resolution. It accepts a `nav_msgs/msg/OccupancyGrid` on the configured topic,
where values from `0` to `100` are converted to costs from `0` to
`max_cost` (default `200`). The resulting grid is merged into Nav2 with
`updateWithMax()`.

The plugin is exported as:

```text
sfg_go2_rssi_costmap_plugin::RssiLayer
```

The initial configuration is in
`sfg_go2_bringup/config/nav2_params.yaml`. The default topic is
`/global/sfg_go2_01/qos_map`.

Alternatively, `map_file` can point to a CSV file with this format:

```text
width,height,resolution,origin_x,origin_y
0,10,20,...
...
```

The following `width * height` values are read in row-major order. Values are
`-1` for unknown or `0..100` for QoS cost.

The map generator in `sfg_go2_hardware_interface` consumes the same CSV
format. It loads the foundation map at startup, publishes it latched on
`/global/sfg_go2_01/qos_map`, and replaces the cell at the current robot pose
whenever a new RSSI value arrives on `/local/sfg_go2_01/rssi`.
