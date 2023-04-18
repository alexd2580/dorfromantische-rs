from enum import Enum
from dataclasses import dataclass
from typing import Optional
import sys


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
    """Clockwise!"""

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

    unknown_102 = 102
    unknown_105 = 105
    water_size4 = 109 # wtf?
    unknown_111 = 111

    @staticmethod
    def from_dump(data):
        assert data["__class"] == "Dorfromantik.SegmentTypeId"
        return Form(data["value__"])

@dataclass
class Segment:
    terrain: Terrain
    form: Form
    rotation: int

    @staticmethod
    def from_dump(data):
        assert data["__class"] == "SegmentData002"
        return Segment(
            Terrain.from_dump(data["groupType"]),
            Form.from_dump(data["segmentType"]),
            data["rotation"]
        )

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

    rotation: int
    segments: list[Segment]
    special_tile: SpecialTile

    quest: Optional[Quest]

    @staticmethod
    def from_dump(data):
        assert data["__class"] == "TileData_003"
        pos = data["gridPos"]
        segments = data["segments"]
        quest = data["questTileData"]
        return Tile(
            pos[0],
            pos[1],
            data["rotation"],
            [] if segments is None else [Segment.from_dump(segment) for segment in segments if segment is not None],
            SpecialTile.from_dump(data["specialTileId"]),
            None if quest is None else Quest.from_dump(quest)
        )


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

    @staticmethod
    def from_dump(data):
        assert data["__class"] == "SaveGameData_003"

        tiles = [Tile(0, 0, 0, [], SpecialTile(0), None)]
        tiles.extend([Tile.from_dump(tile) for tile in data["tiles"] if tile is not None])

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
            [PreplacedTile.from_dump(tile) for tile in data["preplacedTiles"] if tile is not None],
            [Tile.from_dump(tile) for tile in data["tileStack"] if tile is not None],
            tiles,
        )


if __name__ == "__main__":
    from dorfromantischer.savegame import parse_savegame
    from pprint import pprint

    pprint(State.from_dump(parse_savegame(sys.argv[1:])))
