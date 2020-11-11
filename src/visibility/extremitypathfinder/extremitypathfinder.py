import pickle
from copy import deepcopy
from typing import Iterable, List, Optional, Tuple, Union

import numpy as np

from extremitypathfinder.helper_classes import DirectedHeuristicGraph, Edge, Polygon, PolygonVertex, Vertex
from extremitypathfinder.helper_fcts import (
    check_data_requirements, convert_gridworld, find_visible, find_within_range, inside_polygon,
)

# TODO possible to allow polygon consisting of 2 vertices only(=barrier)? lots of functions need at least 3 vertices atm

COORDINATE_TYPE = Tuple[float, float]
PATH_TYPE = List[COORDINATE_TYPE]
LENGTH_TYPE = Optional[float]
INPUT_NUMERICAL_TYPE = Union[float, int]
INPUT_COORD_TYPE = Tuple[INPUT_NUMERICAL_TYPE, INPUT_NUMERICAL_TYPE]
OBSTACLE_ITER_TYPE = Iterable[INPUT_COORD_TYPE]
INPUT_COORD_LIST_TYPE = Union[np.ndarray, List]

DEFAULT_PICKLE_NAME = 'environment.pickle'


# is not a helper function to make it an importable part of the package
def load_pickle(path=DEFAULT_PICKLE_NAME):
    print('loading map from:', path)
    with open(path, 'rb') as f:
        return pickle.load(f)


# TODO document parameters
class PolygonEnvironment:
    """ class allowing to use polygons to represent "2D environments" and use them for path finding.

    keeps a "loaded" and prepared environment for consecutive path queries.
    internally uses a visibility graph optimised for shortest path finding.
    general approach and some optimisations theoretically described in:
    [1] Vinther, Anders Strand-Holm, Magnus Strand-Holm Vinther, and Peyman Afshani.
    "`Pathfinding in Two-dimensional Worlds
    <https://www.cs.au.dk/~gerth/advising/thesis/anders-strand-holm-vinther_magnus-strand-holm-vinther.pdf>`__"
    """

    boundary_polygon: Polygon = None
    holes: List[Polygon] = None
    prepared: bool = False
    graph: DirectedHeuristicGraph = None
    temp_graph: DirectedHeuristicGraph = None  # for storing and plotting the graph during a query

    @property
    def polygons(self) -> Iterable[Polygon]:
        yield self.boundary_polygon
        yield from self.holes

    @property
    def all_vertices(self) -> List[PolygonVertex]:
        for p in self.polygons:
            yield from p.vertices

    @property
    def all_extremities(self) -> Iterable[PolygonVertex]:
        for p in self.polygons:
            yield from p.extremities

    @property
    def all_edges(self) -> Iterable[Edge]:
        for p in self.polygons:
            yield from p.edges

    def store(self, boundary_coordinates: INPUT_COORD_LIST_TYPE, list_of_hole_coordinates: INPUT_COORD_LIST_TYPE,
              validate: bool = False):
        """ saves the passed input polygons in the environment

        .. note:: the passed polygons must meet these requirements:

            * given as numpy or python array of coordinate tuples: ``[(x1,y1), (x2,y2,)...]``
            * no repetition of the first point at the end
            * at least 3 vertices (no single points or lines allowed)
            * no consequent vertices with identical coordinates in the polygons (same coordinates allowed)
            * no self intersections
            * no true intersections with other polygons, identical vertices allowed
            * edge numbering has to follow these conventions: boundary polygon counter clockwise, holes clockwise

        :param boundary_coordinates: array of coordinates with counter clockwise edge numbering
        :param list_of_hole_coordinates: array of coordinates with clockwise edge numbering
        :param validate: whether the requirements of the data should be tested

        :raises AssertionError: when validate=True and the input is invalid.
        """
        self.prepared = False
        # 'loading the map
        boundary_coordinates = np.array(boundary_coordinates)
        list_of_hole_coordinates = [np.array(hole_coords) for hole_coords in list_of_hole_coordinates]
        if validate:
            check_data_requirements(boundary_coordinates, list_of_hole_coordinates)

        self.boundary_polygon = Polygon(boundary_coordinates, is_hole=False)
        # IMPORTANT: make a copy of the list instead of linking to the same list (python!)
        self.holes = [Polygon(coordinates, is_hole=True) for coordinates in list_of_hole_coordinates]

    def store_grid_world(self, size_x: int, size_y: int, obstacle_iter: OBSTACLE_ITER_TYPE, simplify: bool = True,
                         validate: bool = False):
        """ convert a grid-like into a polygon environment and save it

        prerequisites: grid world must not have single non-obstacle cells which are surrounded by obstacles
        ("white cells in black surrounding" = useless for path planning)

        :param size_x: the horizontal grid world size
        :param size_y: the vertical grid world size
        :param obstacle_iter: an iterable of coordinate pairs (x,y) representing blocked grid cells (obstacles)
        :param validate: whether the input should be validated
        :param simplify: whether the polygons should be simplified or not. reduces edge amount, allow diagonal edges
        """
        boundary_coordinates, list_of_hole_coordinates = convert_gridworld(size_x, size_y, obstacle_iter, simplify)
        self.store(boundary_coordinates, list_of_hole_coordinates, validate)

    def export_pickle(self, path: str = DEFAULT_PICKLE_NAME):
        print('storing map class in:', path)
        with open(path, 'wb') as f:
            pickle.dump(self, f)
        print('done.\n')

    def translate(self, new_origin: Vertex):
        """ shifts the coordinate system to a new origin

        computing the angle representations, shifted coordinates and distances for all vertices
        respective to the query point (lazy!)

        :param new_origin: the origin of the coordinate system to be shifted to
        """
        for p in self.polygons:
            p.translate(new_origin)

    def prepare(self):
        """ computes a visibility graph optimized (=reduced) for path planning and stores it

        computes all directly reachable extremities based on visibility and their distance to each other

        .. note::
            multiple polygon vertices might have identical coordinates.
            they must be treated as distinct vertices here, since their attached edges determine visibility
            in the created graph however these nodes must be merged at the end to avoid ambiguities!

        .. note::
            pre computing the shortest paths between all directly reachable extremities
            and storing them in the graph would not be an advantage, because then the graph is fully connected
            a star would visit every node in the graph at least once (-> disadvantage!).
        """

        if self.prepared:
            raise ValueError('this environment is already prepared. load new polygons first.')

        # preprocessing the map
        # construct graph of visible (=directly reachable) extremities
        # and optimize graph further at construction time
        self.graph = DirectedHeuristicGraph()
        extremities_to_check = set(self.all_extremities)
        # have to run for all (also last one!), because existing edges might get deleted every loop
        while len(extremities_to_check) > 0:
            # extremities are always visible to each other (bi-directional relation -> undirected graph)
            #  -> do not check extremities which have been checked already
            #  (would only give the same result when algorithms are correct)
            # the extremity itself must not be checked when looking for visible neighbours
            query_extremity: PolygonVertex = extremities_to_check.pop()
            # CHANGE: do not consider vertexed outside map
            if not self.within_map(query_extremity.coordinates):
                continue
            self.translate(new_origin=query_extremity)

            visible_vertices = set()
            candidate_extremities = extremities_to_check.copy()
            # remove the extremities with the same coordinates as the query extremity
            candidate_extremities.difference_update(
                {c for c in candidate_extremities if c.get_angle_representation() is None})

            # these vertices all belong to a polygon
            # direct neighbours of the query vertex are visible
            # neighbouring vertices are reachable with the distance equal to the edge length
            n1, n2 = query_extremity.get_neighbours()
            # CHANGE: Don't assume neighbouring nodes are visible, might be blocked by intersecting polygon
            '''try:
                candidate_extremities.remove(n1)

                visible_vertices.add((n1, n1.get_distance_to_origin()))
            except KeyError:
                pass
            try:
                candidate_extremities.remove(n2)
                
                visible_vertices.add((n2, n2.get_distance_to_origin()))
            except KeyError:
                pass'''

            # even though candidate_extremities might be empty now
            # must not skip to next loop here, because existing graph edges might get deleted here!

            # eliminate all vertices 'behind' the query point from the candidate set
            # since the query vertex is an extremity the 'outer' angle is < 180 degree
            # then the difference between the angle representation of the two edges has to be < 2.0
            # all vertices between the angle of the two neighbouring edges ('outer side')
            #   are not visible (no candidates!)
            # vertices with the same angle representation might be visible! do not delete them!
            repr1 = n1.get_angle_representation()
            repr2 = n2.get_angle_representation()
            repr_diff = abs(repr1 - repr2)
            candidate_extremities.difference_update(
                find_within_range(repr1, repr2, repr_diff, candidate_extremities, angle_range_less_180=True,
                                  equal_repr_allowed=False))

            # as shown in [1, Ch. II 4.4.2 "Property One"] Starting from any point lying "in front of" an extremity e,
            # such that both adjacent edges are visible, one will never visit e, because everything is
            # reachable on a shorter path without e (except e itself). An extremity e1 lying in the area "in front of"
            #   extremity e hence is never the next vertex in a shortest path coming from e.
            #   And also in reverse: when coming from e1 everything else than e itself can be reached faster
            #   without visiting e2. -> e1 and e do not have to be connected in the graph.
            # IMPORTANT: this condition only holds for building the basic visibility graph!
            #   when a query point happens to be an extremity, edges to the (visible) extremities in front
            #   MUST be added to the graph!
            # find extremities which fulfill this condition for the given query extremity
            repr1 = (repr1 + 2.0) % 4.0  # rotate 180 deg
            repr2 = (repr2 + 2.0) % 4.0
            # IMPORTANT: the true angle diff does not change, but the repr diff does! compute again
            repr_diff = abs(repr1 - repr2)

            # IMPORTANT: check all extremities here, not just current candidates
            # do not check extremities with equal coordinates (also query extremity itself!)
            #   and with the same angle representation (those edges must not get deleted from graph!)
            temp_candidates = set(filter(lambda e: e.get_angle_representation() is not None, self.all_extremities))
            lie_in_front = find_within_range(repr1, repr2, repr_diff, temp_candidates, angle_range_less_180=True,
                                             equal_repr_allowed=False)

            # already existing edges in the graph to the extremities in front have to be removed
            self.graph.remove_multiple_undirected_edges(query_extremity, lie_in_front)
            # do not consider when looking for visible extremities (NOTE: they might actually be visible!)
            candidate_extremities.difference_update(lie_in_front)

            # all edges except the neighbouring edges (handled above!) have to be checked
            edges_to_check = set(self.all_edges)
            edges_to_check.remove(query_extremity.edge1)
            edges_to_check.remove(query_extremity.edge2)

            visible_vertices.update(find_visible(candidate_extremities, edges_to_check))
            # CHANGE: only add if visible vertices are within map
            for v, d in visible_vertices:
                if self.within_map(v.coordinates):
                    self.graph.add_undirected_edge(query_extremity, v, d)

        # join all nodes with the same coordinates
        self.graph.make_clean()
        self.prepared = True

    # make sure start and goal are within the boundary polygon and outside of all holes
    def within_map(self, coords: INPUT_COORD_TYPE):
        """ checks if the given coordinates lie within the boundary polygon and outside of all holes

        :param coords: numerical tuple representing coordinates
        :return: whether the given coordinate is a valid query point
        """
        #
        x, y = coords
        if not inside_polygon(x, y, self.boundary_polygon.coordinates, border_value=True):
            return False
        for hole in self.holes:
            if inside_polygon(x, y, hole.coordinates, border_value=False):
                return False
        return True

    def find_shortest_path(self, start_coordinates: INPUT_COORD_TYPE, goal_coordinates: INPUT_COORD_TYPE,
                           free_space_after: bool = True, verify: bool = True) -> Tuple[PATH_TYPE, LENGTH_TYPE]:
        """ computes the shortest path and its length between start and goal node

        :param start_coordinates: a (x,y) coordinate tuple representing the start node
        :param goal_coordinates:  a (x,y) coordinate tuple representing the goal node
        :param free_space_after: whether the created temporary search graph self.temp_graph
            should be deleted after the query
        :param verify: whether it should be checked if start and goal points really lie inside the environment.
         if points close to or on polygon edges should be accepted as valid input, set this to ``False``.
        :return: a tuple of shortest path and its length. ([], None) if there is no possible path.
        """
        # path planning query:
        # make sure the map has been loaded and prepared
        if self.boundary_polygon is None:
            raise ValueError('No Polygons have been loaded into the map yet.')
        if not self.prepared:
            self.prepare()

        if verify and not (self.within_map(start_coordinates) and self.within_map(goal_coordinates)):
            raise ValueError('start or goal do not lie within the map')

        if start_coordinates == goal_coordinates:
            # start and goal are identical and can be reached instantly
            return [start_coordinates, goal_coordinates], 0.0

        # could check if start and goal nodes have identical coordinates with one of the vertices
        # optimisations for visibility test can be made in this case:
        # for extremities the visibility has already been (except for in front) computed
        # BUT: too many cases possible: e.g. multiple vertices identical to query point...
        # -> always create new query vertices
        # include start and goal vertices in the graph
        start_vertex = Vertex(start_coordinates)
        goal_vertex = Vertex(goal_coordinates)

        # check the goal node first (earlier termination possible)
        self.translate(new_origin=goal_vertex)  # do before checking angle representations!
        # IMPORTANT: manually translate the start vertex, because it is not part of any polygon
        #   and hence does not get translated automatically
        start_vertex.mark_outdated()

        # the visibility of only the graphs nodes has to be checked (not all extremities!)
        # points with the same angle representation should not be considered visible
        # (they also cause errors in the algorithms, because their angle repr is not defined!)
        candidates = set(filter(lambda n: n.get_angle_representation() is not None, self.graph.get_all_nodes()))
        # IMPORTANT: check if the start node is visible from the goal node!
        candidates.add(start_vertex)

        visibles_n_distances_goal = find_visible(candidates, edges_to_check=set(self.all_edges))
        if len(visibles_n_distances_goal) == 0:
            # The goal node does not have any neighbours. Hence there is not possible path to the goal.
            return [], None

        # create temporary graph TODO make more performant, avoid real copy
        # DirectedHeuristicGraph implements __deepcopy__() to not change the original precomputed self.graph
        # but to still not create real copies of vertex instances!
        self.temp_graph = deepcopy(self.graph)

        # IMPORTANT geometrical property of this problem: it is always shortest to directly reach a node
        #   instead of visiting other nodes first (there is never an advantage through reduced edge weight)
        # -> when goal is directly reachable, there can be no other shorter path to it. Terminate

        for v, d in visibles_n_distances_goal:
            if v == start_vertex:
                return [start_coordinates, goal_coordinates], d

            # add unidirectional edges to the temporary graph
            # add edges in the direction: extremity (v) -> goal
            # TODO: improvement: add edges last, after filtering them. instead of deleting edges
            if self.within_map(v.coordinates):
                self.temp_graph.add_directed_edge(v, goal_vertex, d)

        self.translate(new_origin=start_vertex)  # do before checking angle representations!
        # the visibility of only the graphs nodes have to be checked
        # the goal node does not have to be considered, because of the earlier check
        candidates = set(filter(lambda n: n.get_angle_representation() is not None, self.graph.get_all_nodes()))
        visibles_n_distances_start = find_visible(candidates, edges_to_check=set(self.all_edges))
        if len(visibles_n_distances_start) == 0:
            # The start node does not have any neighbours. Hence there is not possible path to the goal.
            return [], None

        # add edges in the direction: start -> extremity
        # TODO: improvement: add edges last, after filtering them. instead of deleting edges
        self.temp_graph.add_multiple_directed_edges(start_vertex, visibles_n_distances_start)

        # also here unnecessary edges in the graph can be deleted when start or goal lie in front of visible extremities
        # IMPORTANT: when a query point happens to coincide with an extremity, edges to the (visible) extremities
        #  in front MUST be added to the graph! Handled by always introducing new (non extremity, non polygon) vertices.

        # for every extremity that is visible from either goal or start
        # NOTE: edges are undirected! self.temp_graph.get_neighbours_of(start_vertex) == set()
        # neighbours_start = self.temp_graph.get_neighbours_of(start_vertex)
        neighbours_start = {n for n, d in visibles_n_distances_start}
        # the goal vertex might be marked visible, it is not an extremity -> skip
        neighbours_start.discard(goal_vertex)
        neighbours_goal = self.temp_graph.get_neighbours_of(goal_vertex)
        for vertex in neighbours_start | neighbours_goal:
            # assert type(vertex) == PolygonVertex and vertex.is_extremity

            # check only if point is visible
            temp_candidates = set()
            if vertex in neighbours_start:
                temp_candidates.add(start_vertex)

            if vertex in neighbours_goal:
                temp_candidates.add(goal_vertex)

            if len(temp_candidates) > 0:
                self.translate(new_origin=vertex)
                # IMPORTANT: manually translate the goal and start vertices
                start_vertex.mark_outdated()
                goal_vertex.mark_outdated()

                n1, n2 = vertex.get_neighbours()
                repr1 = (n1.get_angle_representation() + 2.0) % 4.0  # rotated 180 deg
                repr2 = (n2.get_angle_representation() + 2.0) % 4.0
                repr_diff = abs(repr1 - repr2)

                # IMPORTANT: special case:
                # here the nodes must stay connected if they have the same angle representation!
                lie_in_front = find_within_range(repr1, repr2, repr_diff, temp_candidates, angle_range_less_180=True,
                                                 equal_repr_allowed=False)
                self.temp_graph.remove_multiple_undirected_edges(vertex, lie_in_front)

        # NOTE: exploiting property 2 from [1] here would be more expensive than beneficial
        vertex_path, distance = self.temp_graph.modified_a_star(start_vertex, goal_vertex)

        if free_space_after:
            del self.temp_graph  # free the memory

        # extract the coordinates from the path
        return [tuple(v.coordinates) for v in vertex_path], distance


if __name__ == "__main__":
    # TODO command line support. read polygons and holes from .json files?
    pass
