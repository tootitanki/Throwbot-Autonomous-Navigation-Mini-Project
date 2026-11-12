"""Single source of truth for the sensor models: builds xacro args, the Nav2 parameter file and the bridge file."""
import copy, math, os, tempfile
import yaml
from ament_index_python.packages import get_package_share_directory as share

SENSORS = ['lidar', 'lidar_fwd87', 'depth', 'depth160', 'ultrasonic']


def load_sensors():
    with open(os.path.join(share('throwbot_sensors'), 'config', 'sensors.yaml')) as f:
        return yaml.safe_load(f)


def xacro_args(name):
    s = load_sensors()[name]
    a = {'sensor_kind': s['kind'], 'rate': s['rate_hz'], 'noise': s['noise_std'],
         'rmin': s['range_min'], 'rmax': s['range_max'], 'mx': s['mount']['x'], 'mz': s['mount']['z']}
    if s['kind'] == 'depth':
        a.update(width=s['width'], height=s['height'], hfov=s['hfov'])
    else:
        a.update(samples=s['samples'], amin=s['min_angle'], amax=s['max_angle'])
    return ' '.join(f'{k}:={v}' for k, v in a.items())


def _layer(s, local):
    c = s['costmap']
    if c['layer'] == 'range':
        return 'range_layer', {
            'plugin': 'nav2_costmap_2d::RangeSensorLayer', 'enabled': True,
            'topics': [c['topic']], 'phi': 1.2, 'inflate_cone': 1.0, 'no_readings_timeout': 0.0,
            'clear_threshold': 0.2, 'mark_threshold': 0.8, 'clear_on_max_reading': True,
            'input_sensor_type': 'ALL'}
    src = {'topic': c['topic'], 'data_type': c['data_type'], 'clearing': True, 'marking': True,
           'max_obstacle_height': 2.0, 'min_obstacle_height': c.get('min_obstacle_height', 0.0),
           'obstacle_max_range': c['obstacle_max_range'], 'raytrace_max_range': c['raytrace_max_range']}
    if local:
        return 'voxel_layer', {'plugin': 'nav2_costmap_2d::VoxelLayer', 'enabled': True,
                               'observation_sources': 'obs', 'obs': src}
    return 'obstacle_layer', {'plugin': 'nav2_costmap_2d::ObstacleLayer', 'enabled': True,
                              'observation_sources': 'obs', 'obs': src}


def nav2_params_file(name, outdir):
    s = load_sensors()[name]
    base = os.path.join(share('throwbot_sensors'), 'config', 'nav2.yaml')
    with open(base) as f:
        p = yaml.safe_load(f)
    lc = p['local_costmap']['local_costmap']['ros__parameters']
    gc = p['global_costmap']['global_costmap']['ros__parameters']
    for k in ('voxel_layer', 'obstacle_layer', 'range_layer'):
        lc.pop(k, None); gc.pop(k, None)
    ln, lv = _layer(s, True)
    gn, gv = _layer(s, False)
    lc['plugins'] = [ln, 'inflation_layer']; lc[ln] = lv
    gc['plugins'] = ['static_layer', gn, 'inflation_layer']; gc[gn] = gv
    gc['track_unknown_space'] = False      # fully known static map (fixed room); only the moving box is unknown
    lc['publish_frequency'] = 5.0          # costmap is logged for detection timing
    out = os.path.join(outdir, f'nav2_{name}.yaml')
    with open(out, 'w') as f:
        yaml.safe_dump(p, f)
    return out


def bridge_file(name, outdir):
    s = load_sensors()[name]
    with open(os.path.join(share('throwbot_sensors'), 'config', 'bridge.yaml')) as f:
        b = [e for e in yaml.safe_load(f) if e['ros_topic_name'] != '/scan']
    if s['kind'] == 'depth':
        b.append({'ros_topic_name': '/sensor/depth/points', 'gz_topic_name': '/sensor/depth/points',
                  'ros_type_name': 'sensor_msgs/msg/PointCloud2', 'gz_type_name': 'ignition.msgs.PointCloudPacked',
                  'direction': 'GZ_TO_ROS'})
    else:
        b.append({'ros_topic_name': '/sensor/scan', 'gz_topic_name': '/sensor/scan',
                  'ros_type_name': 'sensor_msgs/msg/LaserScan', 'gz_type_name': 'ignition.msgs.LaserScan',
                  'direction': 'GZ_TO_ROS'})
    out = os.path.join(outdir, f'bridge_{name}.yaml')
    with open(out, 'w') as f:
        yaml.safe_dump(b, f)
    return out


def gen_dir():
    return tempfile.mkdtemp(prefix='throwbot_sensors_')
