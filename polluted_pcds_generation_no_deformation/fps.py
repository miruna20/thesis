import numpy as np
import random
from scipy import spatial


def fps_points(points_np, num_samples, return_indices=False):

    seeds = [np.array([0.0, 0.0, 0.0])]
    nn_index = spatial.cKDTree(seeds)
    dists, _ = nn_index.query(points_np, k=1)
    inds = []
    for i in range(num_samples):
        new_idx = np.argmax(dists)
        sample = points_np[new_idx]
        seeds.append(points_np[new_idx])

        dists[new_idx] = -1
        dists = np.minimum(dists, np.linalg.norm(points_np - sample, axis=1))

        inds.append(new_idx)
    seeds.pop(0)
    seeds = np.array(seeds)

    if return_indices:
        return seeds, inds
    else:
        return seeds


def fps_points_random(points_np, num_samples, return_indices=False):

    rand_seeds = np.random.choice(points_np.shape[0], num_samples // 2)
    seeds = [points_np[id, :] for id in rand_seeds]
    nn_index = spatial.cKDTree(seeds)
    dists, _ = nn_index.query(points_np, k=1)
    inds = list(rand_seeds)
    for i in range(num_samples // 2):
        new_idx = np.argmax(dists)
        sample = points_np[new_idx]
        seeds.append(points_np[new_idx])

        dists[new_idx] = -1
        dists = np.minimum(dists, np.linalg.norm(points_np - sample, axis=1))

        inds.append(new_idx)
    # seeds.pop(0)
    seeds = np.array(seeds)

    if return_indices:
        return seeds, inds
    else:
        return seeds

