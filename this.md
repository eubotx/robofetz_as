for node in $(ros2 node list); do   echo "=== $node ===";   ros2 param get $node use_sim_time 2>/dev/null || echo "No use_sim_time param"; done



Services are generated for every ros2 node that uses parameters.
    /describe_parameters - Get detailed descriptions of parameters

    /get_parameter_types - Get the data types of parameters

    /get_parameters - Get current parameter values

    /list_parameters - List all available parameters

    /set_parameters - Set one or more parameters

    /set_parameters_atomically - Set multiple parameters as one atomic operation



ros2 run tf2_tools view_frames