
import numpy as np
from sklearn.neighbors import NearestNeighbors

class LikelihoodField(object):


    def __init__(self, map):
        # grab the map from the map server
        self.map = map

        # The coordinates of each grid cell in the map
        X = np.zeros((self.map.info.width * self.map.info.height, 2))

        # while we're at it let's count the number of occupied cells
        total_occupied = 0
        curr = 0
        for i in range(self.map.info.width):
            for j in range(self.map.info.height):
                # occupancy grids are stored in row major order
                ind = i + j * self.map.info.width
                if self.map.data[ind] > 0:
                    total_occupied += 1
                X[curr, 0] = float(i)
                X[curr, 1] = float(j)
                curr += 1

        # The coordinates of each occupied grid cell in the map
        occupied = np.zeros((total_occupied, 2))
        curr = 0
        for i in range(self.map.info.width):
            for j in range(self.map.info.height):
                # occupancy grids are stored in row major order
                ind = i + j * self.map.info.width
                if self.map.data[ind] > 0:
                    occupied[curr, 0] = float(i)
                    occupied[curr, 1] = float(j)
                    curr += 1

        self.closest_occ = np.zeros(
            (self.map.info.width, self.map.info.height))
        # TODO
        # Get the closest obstacle distance for each coordinate (set the value of self.closest_occ properly)
        # Hint: Nearest Neighbor Algorithm
        # Note that the distance should be the real world distance, not the distance on the map
        nnb = NearestNeighbors(n_neighbors=1, algorithm='auto').fit(occupied)
        distances, indices = nnb.kneighbors(X)
        # to make its distance be the real world have to multiple the self.map.info.resolution
        self.closest_occ = distances.reshape(self.map.info.height, self.map.info.width) * self.map.info.resolution

    def get_closest_obstacle_distance(self, x, y):
        """ Compute the closest obstacle to the specified (x,y) coordinate in
            the map.  If the (x,y) coordinate is out of the map boundaries, nan
            will be returned. """
        x_coord = (x -
                   self.map.info.origin.position.x) / self.map.info.resolution
        y_coord = (y -
                   self.map.info.origin.position.y) / self.map.info.resolution
        if type(x) is np.ndarray:
            x_coord = x_coord.astype(np.int)
            y_coord = y_coord.astype(np.int)
        else:
            x_coord = int(x_coord)
            y_coord = int(y_coord)

        # TODO
        # check if the coordinates are valid, then
        # use self.closest_occ to return the closest obstacle distance
        if x_coord < 0 or x_coord >= self.map.info.width or y_coord < 0 or y_coord >= self.map.info.height:
            return np.nan

        if self.closest_occ[x_coord, y_coord] > 0:
            return self.closest_occ[x_coord, y_coord]

        min_distance = np.inf

        for i in range(occupied.shape[0]):
            # use np function to find the nearest nodes
            distance = np.linalg.norm(np.array([x_coord, y_coord]) - occupied[i])

        if distance < min_distance:
            min_distance = distance

        self.closest_occ[x_coord, y_coord] = min_distance

        return min_distance