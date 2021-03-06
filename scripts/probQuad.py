import numpy as np


class Node:
    def __init__(self, x_min, x_max, y_min, y_max):
        self.pn = 0.5
        self.beta = 0.1
        self.alpha = 0.1
        self.visited = False

        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max


class QuadTree:
    def __init__(self, x_min, x_max, y_min, y_max, max_depth=10, _depth=0):
        self.nodes = []
        self.children = []
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max
        self._depth = _depth
        self.prob = 0.5

    def _split(self, bbox, ref=None, route=None):
        # find cell x_min, x_max, y_min, y_max
        x_min, x_max, y_min, y_max = bbox
        width = x_max - x_min
        height = y_max - y_min

        halfwidth = width / 2.0
        halfheight = height / 2.0

        if ref is None:
            new_depth = self._depth + 1
            self.children = [QuadTree(x_min, x_min + halfwidth, y_min, y_min + halfheight, _depth=new_depth),
                             QuadTree(x_min+halfwidth, x_max, y_min, y_min+halfheight, _depth=new_depth),
                             QuadTree(x_min, x_min + halfwidth, y_min+halfheight, y_max, _depth=new_depth),
                             QuadTree(x_min+halfwidth, x_max, y_min+halfheight, y_max, _depth=new_depth)]

            pn = self.prob
            for node in self.children:
                node.prob = pn
        else:
            new_depth = len(route)+1
            ref.children = [QuadTree(x_min, x_min + halfwidth, y_min, y_min + halfheight, _depth=new_depth),
                             QuadTree(x_min+halfwidth, x_max, y_min, y_min+halfheight, _depth=new_depth),
                             QuadTree(x_min, x_min + halfwidth, y_min+halfheight, y_max, _depth=new_depth),
                             QuadTree(x_min+halfwidth, x_max, y_min+halfheight, y_max, _depth=new_depth)]

            pn = ref.prob
            for node in ref.children:
                node.prob = pn

    def _find_cell(self, location):
        results = None
        level = 0
        quarter = -1
        if len(self.children) == 0:
            # First case (only root node)
            level = 0
            return None, level, quarter

        quarter = 0
        route = []
        children = self.children

        has_children = True
        while has_children:
            for child in children:
                if (child.x_min <= location[0] and child.x_max >= location[0] and \
                                child.y_min <= location[1] and child.y_max >= location[1]):
                    if len(child.children) == 0:
                        # No children this is the leaf node
                        results = child
                        level = child._depth
                        route.append(quarter)
                        has_children = False
                        return results, level, route
                    else:
                        children = child.children
                        route.append(quarter)
                        quarter = 0
                        break
                quarter += 1

        return results, level, route



class Root(QuadTree):
    def __init__(self, bbox, max_depth=10):
        x_min, x_max, y_min, y_max = bbox
        QuadTree.__init__(self, x_min, x_max, y_min, y_max, max_depth)

    def find_cell(self, location):
        return self._find_cell(location)

    def add_measurement(self, measurement, location, level):
        c_level = -1
        while c_level < level:
            results, c_level, route = self._find_cell(location)
            print(c_level)
            if results is None:
                bbox = (self.x_min, self.x_max, self.y_min, self.y_max)
                self._split(bbox)
            elif c_level > level:
                print('Currently expecting a more Detailed Resolution!!!!!!!!!!!!!!!!!!!')
                break
            elif c_level < level:
                bbox = (results.x_min, results.x_max, results.y_min, results.y_max)
                self._split(bbox, results, route)
            if c_level == level:
                results.prob = measurement

    def merge_trees(self, tree2):
        test = True
        t1_children = self.children
        t2_children = tree2.children
        while test:
            for node in t1_children:
                print('TODO')

        print('TODO')

    def add_measurement_xyz(self, measurement, location_xyz):
        location_xy = location_xyz[0:2]
        level = 1 # TODO compute the level
        self.add_measurement(measurement, location_xy, level)


def get_grid_size(area, max_grid):
    side = np.sqrt(area)
    len_grid = np.sqrt(max_grid)

    num_bins = int(side/len_grid) + 1
    len_bin = side/num_bins

    return len_bin, num_bins


class Map:
    def __init__(self, bin_len, bin_nums):
        self.bin_len = bin_len
        self.num_bins = bin_nums
        self.q_tree = []
        for i in range(self.num_bins):
            col = []
            for j in range(self.num_bins):
                x_min = i*bin_len
                x_max = (i + 1)*bin_len
                y_min = j*bin_len
                y_max = (j+1)*bin_len
                roots = Root(bbox=(x_min, x_max, y_min, y_max))
                col.append(roots)
            self.q_tree.append(col)

    def find_tree(self, location):
        for i in range(self.num_bins):
            for j in range(self.num_bins):
                result = self.q_tree[i][j].find_cell(location)
                if result is not None:
                    return i, j



if __name__ == '__main__':
    # Assumption: Using a square area and max grid size is also square for ease of use
    total_area = 500
    max_top_grid = 10

    q_tree = Root(bbox=(0, 500, 0, 500))
    q_tree2 = Root(bbox=(0, 500, 0, 500))
    location = (275, 5)
    location2 = (385, 5)
    measurement = .85
    q_tree.add_measurement(0.8, location, level=3)  # TODO: Add more logic for comparing some at different levels
    q_tree2.add_measurement(0.6, location2, level=2)

    q_tree.merge_trees(q_tree2)

    # q_tree.get_Probability(x, y)
    #
    # len_bin, num_bins = get_grid_size(total_area, max_top_grid)
    # q_map = Map(len_bin, num_bins)
    # loc = [21, 2.0]
    #
    # i,j = q_map.find_tree(loc)



    print('Todo')
