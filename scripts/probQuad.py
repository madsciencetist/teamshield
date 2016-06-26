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
        self.prob = 1

    def _split(self, bbox, ref=None):
        # find cell x_min, x_max, y_min, y_max
        x_min, x_max, y_min, y_max = bbox
        width = x_max - x_min
        height = y_max - y_min

        halfwidth = width / 2.0
        halfheight = height / 2.0

        new_depth = self._depth + 1
        if ref is None:
            self.children = [QuadTree(x_min, x_min + halfwidth, y_min, y_min + halfheight, _depth=new_depth),
                             QuadTree(x_min+halfwidth, x_max, y_min, y_min+halfheight, _depth=new_depth),
                             QuadTree(x_min, x_min + halfwidth, y_min+halfheight, y_max, _depth=new_depth),
                             QuadTree(x_min+halfwidth, x_max, y_min+halfheight, y_max, _depth=new_depth)]

            pn = self.prob/4
            for node in self.children:
                node.prob = pn
        else:
            ref.children = [QuadTree(x_min, x_min + halfwidth, y_min, y_min + halfheight, _depth=new_depth),
                             QuadTree(x_min+halfwidth, x_max, y_min, y_min+halfheight, _depth=new_depth),
                             QuadTree(x_min, x_min + halfwidth, y_min+halfheight, y_max, _depth=new_depth),
                             QuadTree(x_min+halfwidth, x_max, y_min+halfheight, y_max, _depth=new_depth)]

            pn = ref.prob/4
            for node in ref.children:
                node.prob = pn



        # nodes = self.nodes
        # self.nodes = []
        # for node in nodes:
        #     self._insert_into_children(node.item, node.rect)

    def _find_cell(self, location):
        results = None
        level = 0
        quarter = -1
        if len(self.children) == 0:
            # First case (only root node)
            level = 0
            return None, level, quarter

        quarter = 0
        for child in self.children:
            if (child.x_min <= location[0] and child.x_max >= location[0] and \
                            child.y_min <= location[1] and child.y_max >= location[1]):
                if len(child.children) == 0:
                    # No children this is the leaf node
                    results = child
                    level = child._depth
                    return results, level, quarter
                else:
                    children = child.children
            quarter += 1

        return results, level, quarter



class Root(QuadTree):
    def __init__(self, bbox, max_depth=10):
        x_min, x_max, y_min, y_max = bbox
        super(Root, self).__init__(x_min, x_max, y_min, y_max, max_depth)

    def find_cell(self, location):
        return self._find_cell(location)

    def add_measurement(self, location, level):
        c_level = -1
        while c_level < level:
            results, c_level = self._find_cell(location)
            if results is None:
                bbox = (self.x_min, self.x_max, self.y_min, self.y_max)
                self._split(bbox)
            elif c_level < level:
                bbox = (results.x_min, results.x_max, results.y_min, results.y_max)
                self._split(bbox, results)






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
    location = (2, 5)

    q_tree.add_measurement(location, level=2)
    # q_tree.get_Probability(x, y)
    #
    # len_bin, num_bins = get_grid_size(total_area, max_top_grid)
    # q_map = Map(len_bin, num_bins)
    # loc = [21, 2.0]
    #
    # i,j = q_map.find_tree(loc)



    print('Todo')