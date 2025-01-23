import os

import matplotlib.pyplot as plt
from PIL import Image
from commonroad.scenario.scenario import Scenario
# commonroad_dc
from commonroad.visualization.mp_renderer import MPRenderer
from matplotlib import pyplot
from planning_generator import PlanningGenerator

result_path = './results/'


def scenario_to_image(scenario: Scenario, planners, step, show_reference_path=False):
    rnd = MPRenderer()
    scenario.draw(rnd, draw_params={
        'time_begin': step})
    for packed_values in planners:
        try:
            planning_generator: PlanningGenerator
            planning_generator, trajectory, ref_path = packed_values
            color = planning_generator.color
            ego_vehicle = planning_generator.planner.convert_cr_trajectory_to_object(
                trajectory)
            ego_vehicle.draw(rnd, draw_params={'time_begin': step, 'dynamic_obstacle': {
                'trajectory': {'facecolor': color, 'line_width': 0.5},
                'vehicle_shape': {'occupancy': {'shape': {'rectangle': {'facecolor': color, 'edgecolor': '#E37222',
                                                                        'zorder': 30, 'opacity': 0.8}}}}}})
            planning_generator.planning_problem.goal.draw(
                rnd, draw_params={'goal_region': {'shape': {'rectangle': {'facecolor': color, 'opacity': 0.5}}}})
        except ValueError:
            # when planner has no solution left we have no values to draw the trajectory
            pass
    rnd.render(show=False)

    for packed_values in planners:
        try:
            planning_generator, trajectory, ref_path = packed_values
            if show_reference_path:
                plt.plot(ref_path[:, 0], ref_path[:, 1], color='g', marker='.', markersize=0.5, zorder=20,
                         linewidth=0.3,
                         label='reference path')
        except ValueError:
            # when planner has no solution left we have no values to draw the trajectory
            pass
    plt.gca().set_aspect('equal')
    plt.autoscale()
    # plt.show()
    canvas = pyplot.get_current_fig_manager().canvas
    canvas.draw()
    im = Image.frombytes('RGB', canvas.get_width_height(),
                         canvas.tostring_rgb())
    im = im.quantize(method=Image.MEDIANCUT)
    return im


def save_images_as_gif(images, sim_path, filename):
    path = result_path + sim_path
    os.makedirs(path, exist_ok=True)
    if len(images) < 1:
        return
    img, *imgs = images
    img.save(fp=path + '/' + filename + '.gif', format='GIF',
             append_images=imgs, save_all=True, loop=0)


def save_images(images, filename):
    images_dir = 'images'
    path = result_path + filename + '/' + images_dir
    os.makedirs(path, exist_ok=True)
    for i, img in enumerate(images):
        img.save(fp=f"{path}/{filename}_{i}.png", format='PNG')
