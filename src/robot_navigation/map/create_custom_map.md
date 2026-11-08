# Create Map

To create a custom map you can adjust the parameters in `src/robot_navigation/map/my_map.yaml` and execute:

```bash
ros2 run robot_navigation create_map
```

After this you can run `map_node` that maps the enemy position to the map and publishes the defence map.

```bash
ros2 run robot_navigation map_node
```