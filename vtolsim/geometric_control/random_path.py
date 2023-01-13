import numpy as np
import matplotlib.pyplot as plt

class RandomPath:
    def __init__(self, seed=0):

        self.rng = np.random.default_rng(seed)

        # possible directions to go in 2D
        self.directions_2D = np.array([
            [-1, 1],
            [0, 1],
            [1, 1],
            [1, 0],
            [1, -1],
            [0, -1],
            [-1, -1],
            [-1, 0]
        ])

        self.next_direction_offset_weights = [
            (0, .7),
            (1, .14),
            (2, .01),
            (-1, .14),
            (-2, .01)
        ]


    def _choose_next_direction(self, prev_direction_idx):
        random_sample = self.rng.random()
        weight_sum = 0.
        for (offset, weight) in self.next_direction_offset_weights:
            weight_sum += weight
            if weight_sum > random_sample:
                break

        choice = (offset + prev_direction_idx) % (len(self.directions_2D))
        if prev_direction_idx > 7 or prev_direction_idx < 0:
            import pdb; pdb.set_trace()
        return choice

    def generate(self, start_loc=np.array([0., 0.]), start_dir=3, npoints=10, maintain_dir=False):
        path = np.zeros((npoints,2))
        path[0,:] = start_loc

        prev_direction = start_dir
        for i in range(npoints-1):
            path[i+1,:] = path[i,:] + self.directions_2D[prev_direction,:]
            if maintain_dir:
                prev_direction = self._choose_next_direction(start_dir)
            else:
                prev_direction = self._choose_next_direction(prev_direction)

        return path

def reduce_path(path):
    # remove points that are in the same direction as previous/next points

    npoints = path.shape[0]

    reduced_path = []
    reduced_path.append(path[0,:])

    for i in range(1, npoints-1):
        prev_dir = path[i,:] - path[i-1,:]
        next_dir = path[i+1,:] - path[i,:]

        if np.array_equal(prev_dir, next_dir):
            continue
        else:
            reduced_path.append(path[i,:])

    reduced_path.append(path[-1,:])

    return np.array(reduced_path)



def plot_n_random_paths(npaths, random_path_generator):

    fig, ax = plt.subplots(figsize=(5,5))

    for i in range(npaths):
        path = random_path_generator.generate(start_loc = np.array([0, 0]), npoints=10, start_dir=3, maintain_dir=True)
        ax.plot(path[:,0], path[:,1], color=f"C{i}")
        reduced_path = reduce_path(path)
        ax.scatter(reduced_path[:,0], reduced_path[:,1], color=f"C{i}")

    ax.set_xlim((-1,10))
    ax.set_ylim((-5,5))
    ax.set_xticks(np.arange(-1,11,1))
    ax.set_yticks(np.arange(-5,6,1))
    ax.set_aspect("equal")
    ax.grid(True)

if __name__ == "__main__":
    random_path_generator = RandomPath(0)
    plot_n_random_paths(5, random_path_generator)
    plt.show()
