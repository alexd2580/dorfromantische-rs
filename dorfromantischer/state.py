from enum import Enum
from dataclasses import dataclass
from typing import Optional
import sys
from collections import deque


class Terrain(Enum):
    empty = -1
    house = 0
    forest = 1
    wheat = 2
    rail = 3
    water = 4

    @staticmethod
    def from_dump(data):
        assert data["__class"] == "GroupTypeId"
        return Terrain(data["value__"])


class Form(Enum):
    """Clockwise! Also, we remap the IDs for better space usage on the GPU."""

    size1 = 1
    size2 = 2
    bridge = 3 # 1-skip1-1
    straight = 4 # 1-skip2-1
    size3 = 5
    junction_left = 6 # 2-skip1-1
    junction_right = 7 # 2-skip2-1
    three_way = 8 # 1-skip1-1-skip1-1
    size4 = 9
    fan_out = 10 # 3-skip1-1
    x = 11 # 2-skip1-2
    size5 = 12
    size6 = 13

    unknown_102 = 14
    unknown_105 = 15
    water_size4 = 16 # wtf?
    unknown_111 = 17

    @staticmethod
    def from_dump(data):
        assert data["__class"] == "Dorfromantik.SegmentTypeId"
        match data["value__"]:
            case 1:
                return Form.size1
            case 2:
                return Form.size2
            case 3:
                return Form.bridge
            case 4:
                return Form.straight
            case 5:
                return Form.size3
            case 6:
                return Form.junction_left
            case 7:
                return Form.junction_right
            case 8:
                return Form.three_way
            case 9:
                return Form.size4
            case 10:
                return Form.fan_out
            case 11:
                return Form.x
            case 12:
                return Form.size5
            case 13:
                return Form.size6
            case 102:
                return Form.unknown_102
            case 105:
                return Form.unknown_105
            case 109:
                return Form.water_size4
            case 111:
                return Form.unknown_111


form_mapping = {
    Form.size1: [0],
    Form.size2: [0, 1],
    Form.bridge: [0, 2],
    Form.straight: [0, 3],
    Form.size3: [0, 1, 2],
    Form.junction_left: [0, 1, 3],
    Form.junction_right: [0, 1, 4],
    Form.three_way: [0, 2, 4],
    Form.size4: [0, 1, 2, 3],
    Form.fan_out: [0, 1, 2, 4],
    Form.x: [0, 1, 3, 4],
    Form.size5: [0, 1, 2, 3, 4],
    Form.size6: [0, 1, 2, 3, 4, 5],
    Form.unknown_102: [],
    Form.unknown_105: [],
    Form.water_size4: [0, 1, 2, 3],
    Form.unknown_111: [],
}


@dataclass
class Segment:
    terrain: Terrain
    form: Form
    rotation: int
    group: Optional[int]

    @staticmethod
    def from_dump(data, tile_rotation):
        assert data["__class"] == "SegmentData002"
        return Segment(
            Terrain.from_dump(data["groupType"]),
            Form.from_dump(data["segmentType"]),
            (tile_rotation + data["rotation"]) % 6,
            None
        )

    def rotations(self) -> list[int]:
        return [(self.rotation + r) % 6 for r in form_mapping[self.form]]


@dataclass
class SpecialTile:
    id: int

    @staticmethod
    def from_dump(data):
        assert data["__class"] == "Dorfromantik.SpecialTileId"
        return SpecialTile(data["value__"])

@dataclass
class Quest:
    @dataclass
    class Id:
        id: int

        @staticmethod
        def from_dump(data):
            assert data["__class"] == "Dorfromantik.QuestId"
            return Quest.Id(data["value__"])

    @dataclass
    class TileId:
        id: int

        @staticmethod
        def from_dump(data):
            assert data["__class"] == "QuestTileId"
            return Quest.TileId(data["value__"])

    active: bool
    id: Id
    level: int
    queue_index: int
    tile_id: TileId

    target_value: int

    @staticmethod
    def from_dump(data):
        assert data["__class"] == "QuestTileData_002"
        return Quest(
            data["questActive"],
            Quest.Id.from_dump(data["questId"]),
            data["questLevel"],
            data["questQueueIndex"],
            Quest.TileId.from_dump(data["questTileId"]),
            data["targetValue"],
        )

@dataclass
class Tile:
    s: int
    t: int

    segments: list[Segment]
    special_tile: SpecialTile

    quest: Optional[Quest]

    @staticmethod
    def from_dump(data):
        assert data["__class"] == "TileData_003"
        pos = data["gridPos"]
        segments = data["segments"]
        quest = data["questTileData"]
        rotation = data["rotation"]
        return Tile(
            pos[0],
            pos[1],
            [] if segments is None else [Segment.from_dump(segment, rotation) for segment in segments if segment is not None],
            SpecialTile.from_dump(data["specialTileId"]),
            None if quest is None else Quest.from_dump(quest)
        )

    @classmethod
    def quadrant_of(cls, s: int, t: int) -> int:
        return (0 if t >= 0 else 3) if s >= 0 else (1 if t >= 0 else 2)

    @classmethod
    def index_of(cls, s: int, t: int) -> int:
        _s = s if s >= 0 else - 1 - s
        _t = t if t >= 0 else - 1 - t
        st = _s + _t
        return int((st + 1) * st / 2) + _t

    @property
    def quadrant(self) -> int:
        return self.quadrant_of(self.s, self.t)

    @property
    def index(self) -> int:
        return self.index_of(self.s, self.t)

    def neighbor_coordinates(self, rotation: int) -> tuple[int, int]:
        if rotation == 0:
            return self.s, self.t + 1
        elif rotation == 3:
            return self.s, self.t - 1

        s_even = self.s % 2 == 0

        if rotation == 1:
            return self.s + 1, self.t + (1 if s_even else 0)
        elif rotation == 2:
            return self.s + 1, self.t - (0 if s_even else 1)
        elif rotation == 4:
            return self.s - 1, self.t - (0 if s_even else 1)
        elif rotation == 5:
            return self.s - 1, self.t + (1 if s_even else 0)

        raise Exception(f"Unexpected rotation {rotation}")

    def segment_at(self, rotation: int) -> Optional[Segment]:
        for segment in self.segments:
            if rotation in segment.rotations():
                return segment


@dataclass
class PreplacedTile:
    section_x: int
    section_y: int

    # TODO

    @staticmethod
    def from_dump(data):
        assert data["__class"] == "PreplacedTileData_002"
        return PreplacedTile(
            data["sectionGridPosX"],
            data["sectionGridPosY"],
        )


@dataclass
class State:
    file_name: str
    playtime: float

    level: int
    score: int
    tile_stack_count: int

    placed_tiles: int
    surrounded_tiles: int
    perfect_placements: int

    quests_fulfilled: int
    quests_failed: int

    preplaced_tiles: list[PreplacedTile]
    tile_stack: list[Tile]

    tiles: list[Tile]
    quadrants: tuple[list[Tile], list[Tile], list[Tile], list[Tile]]

    @staticmethod
    def from_dump(data):
        assert data["__class"] == "SaveGameData_003"

        tiles = [Tile(0, 0, [], SpecialTile(0), None)]
        tiles.extend([Tile.from_dump(tile) for tile in data["tiles"] if tile is not None])
        preplaced_tiles = [PreplacedTile.from_dump(tile) for tile in data["preplacedTiles"] if tile is not None]

        quadrants = ([], [], [], [])

        for tile in tiles:
            q_index = tile.quadrant
            t_index = tile.index
            quadrant = quadrants[q_index]
            if len(quadrant) < t_index + 1:
                quadrant.extend([None] * ((t_index + 1) - len(quadrant)))

            quadrant[t_index] = tile

        return State(
            data["fileName"],
            data["playtime"],
            data["level"],
            data["score"],
            data["tileStackCount"],
            data["placedTileCount"],
            data["surroundedTilesCount"],
            data["perfectPlacements"],
            data["questsFulfilled"],
            data["questsFailed"],
            preplaced_tiles,
            [Tile.from_dump(tile) for tile in data["tileStack"] if tile is not None],
            tiles,
            quadrants,
        )

    def tile_at(self, s: int, t: int) -> Optional[Tile]:
        quadrant = self.quadrants[Tile.quadrant_of(s, t)]
        index = Tile.index_of(s, t)

        if index < len(quadrant):
            return quadrant[index]

    def assign_groups(self):
        # Assign group ids.
        groups = {}
        next_group_index = 0
        processed = set()
        queue: deque[tuple[int, int]] = deque([(0, 0)])

        while len(queue) != 0:
            (s, t) = queue.popleft()
            if not (tile := self.tile_at(s, t)):
                continue

            # For each segment, aka each separate part of a tile.
            for segment_index, segment in enumerate(tile.segments):
                if segment.group is not None:
                    continue

                # Collect neighbor group ids.
                group_ids = set()
                for rotation in segment.rotations():
                    neighbor_st = tile.neighbor_coordinates(rotation)
                    if not (neighbor := self.tile_at(*neighbor_st)):
                        continue

                    opposite_side = (rotation + 3) % 6
                    if not (neighbor_segment := neighbor.segment_at(opposite_side)):
                        continue

                    if neighbor_segment.terrain == segment.terrain:
                        group_ids.add(neighbor_segment.group)

                    # Enqueue neighbors.
                    if neighbor_st not in processed:
                        queue.append(neighbor_st)

                if None in group_ids:
                    group_ids.remove(None)

                # Create, assign and merge.
                if len(group_ids) == 0:
                    segment.group = next_group_index
                    next_group_index += 1
                    groups[segment.group] = set([(s, t, segment_index)])

                if len(group_ids) == 1:
                    segment.group = group_ids.pop()
                    groups[segment.group].add((s, t, segment_index))

                else:
                    segment.group = min(group_ids)
                    group_ids.remove(id)
                    groups[segment.group].append((s, t, segment_index))

                    for other_id in group_ids:
                        for (other_s, other_t, other_index) in groups[other_id]:
                            if other_tile := self.tile_at(other_s, other_t):
                                other_tile.segments[other_index].group = segment.group
                                groups[segment.group].append((other_s, other_t, other_index))

                        del groups[other_id]


if __name__ == "__main__":
    from dorfromantischer.savegame import parse_savegame
    from pprint import pprint

    pprint(State.from_dump(parse_savegame(sys.argv[1:])))
