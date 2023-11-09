use std::collections::HashMap;

use nrbf_rs::value::Value;

fn expected_got<T>(expected: &str, got: &str) -> Result<T, String> {
    Err(format!(
        "Expected {expected}; Got {}",
        &got[..100.min(got.len())]
    ))
}

enum Maybe<T> {
    Just(T),
    Nothing,
}
impl<T> Maybe<T> {
    fn into_option(self) -> Option<T> {
        match self {
            Maybe::Nothing => None,
            Maybe::Just(x) => Some(x),
        }
    }
}
impl<T: for<'a> TryFrom<&'a Value, Error = std::string::String>> TryFrom<&Value> for Maybe<T> {
    type Error = String;

    fn try_from(value: &Value) -> Result<Self, String> {
        match value {
            Value::Null => Ok(Maybe::Nothing),
            _ => T::try_from(value).map(Maybe::Just),
        }
    }
}

fn filter_none<T>(vec: Vec<Maybe<T>>) -> Vec<T> {
    vec.into_iter().filter_map(Maybe::into_option).collect()
}

fn try_object_from<'a>(
    expected_class: &str,
    value: &'a Value,
) -> Result<&'a HashMap<String, Value>, String> {
    if let Value::Object(class_name, values) = value {
        if class_name != expected_class {
            return expected_got(expected_class, class_name);
        }

        Ok(values)
    } else {
        expected_got(expected_class, &value.to_string())
    }
}

fn try_prefix_object_from<'a>(
    class_prefix: &str,
    value: &'a Value,
) -> Result<&'a HashMap<String, Value>, String> {
    if let Value::Object(class_name, values) = value {
        if !class_name.starts_with(class_prefix) {
            return expected_got(class_prefix, class_name);
        }

        Ok(values)
    } else {
        expected_got(class_prefix, &value.to_string())
    }
}

struct GenericList<T>(Vec<T>);
impl<T> Into<Vec<T>> for GenericList<T> {
    fn into(self) -> Vec<T> {
        self.0
    }
}
impl<T: for<'a> TryFrom<&'a Value, Error = String>> TryFrom<&Value> for GenericList<T> {
    type Error = String;

    fn try_from(value: &Value) -> Result<Self, String> {
        let values = try_prefix_object_from("System.Collections.Generic.List`1[", value)?;
        try_key_as::<Vec<T>>(values, "_items").map(GenericList)
    }
}

fn try_key_of<'a>(values: &'a HashMap<String, Value>, key: &str) -> Result<&'a Value, String> {
    values.get(key).ok_or(format!("No {} field in object", key))
}

fn try_key_as<'a, T: TryFrom<&'a Value, Error = std::string::String>>(
    values: &'a HashMap<String, Value>,
    key: &str,
) -> Result<T, String> {
    T::try_from(try_key_of(values, key)?)
        .map_err(|error| format!("While converting key {key}:\n{error}"))
}

fn from_id_object<'a>(expected_class: &str, value: &'a Value) -> Result<i32, String> {
    try_key_as(try_object_from(expected_class, value)?, "value__")
}

#[derive(Debug)]
struct ChallengeId(i32);

impl TryFrom<&Value> for ChallengeId {
    type Error = String;

    fn try_from(value: &Value) -> Result<Self, String> {
        from_id_object("Dorfromantik.ChallengeId", value).map(Self)
    }
}

#[derive(Debug)]
pub struct GroupTypeId(pub i32);

impl TryFrom<&Value> for GroupTypeId {
    type Error = String;

    fn try_from(value: &Value) -> Result<Self, String> {
        from_id_object("GroupTypeId", value).map(Self)
    }
}

#[allow(dead_code)]
#[derive(Debug)]
pub struct Segment {
    pub group_type: GroupTypeId,
    pub segment_type: SegmentTypeId,
    pub rotation: i32,
    version: i32,
}

impl TryFrom<&Value> for Segment {
    type Error = String;

    fn try_from(value: &Value) -> Result<Self, String> {
        let values = try_object_from("SegmentData002", value)?;
        Ok(Self {
            group_type: try_key_as(values, "groupType")?,
            segment_type: try_key_as(values, "segmentType")?,
            rotation: try_key_as(values, "rotation")?,
            version: try_key_as(values, "version")?,
        })
    }
}

#[allow(dead_code)]
#[derive(Debug)]
struct QuestTile {
    quest_tile_id: QuestTileId,
    quest_active: bool,
    quest_queue_index: i32,
    target_value: i32,
    quest_level: i32,
    quest_id: QuestId,
    unlocked_challenge_id: ChallengeId,
    version: i32,
}

impl TryFrom<&Value> for QuestTile {
    type Error = String;

    fn try_from(value: &Value) -> Result<Self, String> {
        let values = try_object_from("QuestTileData_002", value)?;
        Ok(Self {
            quest_tile_id: try_key_as(values, "questTileId")?,
            quest_active: try_key_as(values, "questActive")?,
            quest_queue_index: try_key_as(values, "questQueueIndex")?,
            target_value: try_key_as(values, "targetValue")?,
            quest_level: try_key_as(values, "questLevel")?,
            quest_id: try_key_as(values, "questId")?,
            unlocked_challenge_id: try_key_as(values, "unlockedChallengeId")?,
            version: try_key_as(values, "version")?,
        })
    }
}

#[allow(dead_code)]
#[derive(Debug)]
pub struct Tile {
    pub s: i32,
    pub t: i32,
    pub rotation: i32,
    seed: i32,
    pub segments: Vec<Segment>,
    pub special_tile_id: SpecialTileId,
    quest_tile: Option<QuestTile>,
    version: i32,
}

impl TryFrom<&Value> for Tile {
    type Error = String;

    fn try_from(value: &Value) -> Result<Self, String> {
        let values = try_object_from("TileData_003", value)?;
        let grid_pos: Vec<i32> = try_key_as(values, "gridPos")?;
        Ok(Tile {
            s: grid_pos[0],
            t: grid_pos[1],
            rotation: try_key_as(values, "rotation")?,
            seed: try_key_as(values, "seed")?,
            segments: filter_none(
                try_key_as::<Maybe<GenericList<_>>>(values, "segments")?
                    .into_option()
                    .map(Into::into)
                    .unwrap_or(vec![]),
            ),
            special_tile_id: try_key_as(values, "specialTileId")?,
            quest_tile: try_key_as::<Maybe<_>>(values, "questTileData")?.into_option(),
            version: try_key_as(values, "version")?,
        })
    }
}

#[allow(dead_code)]
#[derive(Debug)]
struct PreplacedTile {
    section_grid_pos_x: i32,
    section_grid_pos_y: i32,
    preplaced_tile_id: QuestTileId,
    version: i32,
}

impl TryFrom<&Value> for PreplacedTile {
    type Error = String;

    fn try_from(value: &Value) -> Result<Self, String> {
        let values = try_object_from("PreplacedTileData_002", value)?;
        Ok(Self {
            section_grid_pos_x: try_key_as(values, "sectionGridPosX")?,
            section_grid_pos_y: try_key_as(values, "sectionGridPosY")?,
            preplaced_tile_id: try_key_as(values, "preplacedTileId")?,
            version: try_key_as(values, "version")?,
        })
    }
}

#[allow(dead_code)]
#[derive(Debug)]
pub struct SaveGame {
    game_mode: GameModeId,
    level: i32,
    score: i32,
    perfect_placements: i32,
    quests_fulfilled: i32,
    quests_failed: i32,
    consecutive_perfect_fits: i32,
    consecutive_placements_without_rotate: i32,
    playtime: f32,
    biome_seed: i32,
    preplaced_tile_seed: i32,
    placed_tile_count: i32,
    generated_tile_count: i32,
    generated_quest_count: i32,
    surrounded_tiles_count: i32,
    pub tiles: Vec<Tile>,
    tile_stack: Vec<Tile>,
    preplaced_tiles: Vec<PreplacedTile>,
    pending_locked_challenges: Vec<ChallengeId>,
    tile_stack_count: i32,
    file_name: Option<String>,
    initial_version: String,
    last_played_version: String,
    last_rewarded_step: Vec<i32>,
    last_rewarded_score: Vec<i32>,
    version: i32,
}

impl TryFrom<&Value> for SaveGame {
    type Error = String;

    fn try_from(value: &Value) -> Result<Self, String> {
        let values = try_object_from("SaveGameData_003", value)?;
        Ok(Self {
            game_mode: try_key_as(values, "gameMode")?,
            level: try_key_as(values, "level")?,
            score: try_key_as(values, "score")?,
            perfect_placements: try_key_as(values, "perfectPlacements")?,
            quests_fulfilled: try_key_as(values, "questsFulfilled")?,
            quests_failed: try_key_as(values, "questsFailed")?,
            consecutive_perfect_fits: try_key_as(values, "consecutivePerfectFits")?,
            consecutive_placements_without_rotate: try_key_as(
                values,
                "consecutivePlacementsWithoutRotate",
            )?,
            playtime: try_key_as(values, "playtime")?,
            biome_seed: try_key_as(values, "biomeSeed")?,
            preplaced_tile_seed: try_key_as(values, "preplacedTileSeed")?,
            placed_tile_count: try_key_as(values, "placedTileCount")?,
            generated_tile_count: try_key_as(values, "generatedTileCount")?,
            generated_quest_count: try_key_as(values, "generatedQuestCount")?,
            surrounded_tiles_count: try_key_as(values, "surroundedTilesCount")?,
            tiles: filter_none(try_key_as::<GenericList<_>>(values, "tiles")?.into()),
            tile_stack: filter_none(try_key_as::<GenericList<_>>(values, "tileStack")?.into()),
            preplaced_tiles: filter_none(
                try_key_as::<GenericList<_>>(values, "preplacedTiles")?.into(),
            ),

            pending_locked_challenges: try_key_as::<GenericList<_>>(
                values,
                "pendingLockedChallenges",
            )?
            .into(),

            // active_challenges: ActiveSessionQuestList,
            // active_challenges: try_key_as(values, "activeChallenges")?,
            tile_stack_count: try_key_as(values, "tileStackCount")?,
            // screenshot: PrimitiveArray<Byte>
            // last_played: PrimitiveArray<I32>
            file_name: try_key_as::<Maybe<_>>(values, "fileName")?.into_option(),
            initial_version: try_key_as(values, "initialVersion")?,
            last_played_version: try_key_as(values, "lastPlayedVersion")?,
            // group_type_configuration: try_key_as::<GenericList<_>>(values, "groupTypeConfiguration")?,
            // excluded_biomes: try_key_as::<GenericList<_>>(values, "excludedBiomes")?,
            // custom_mode_data: ...
            last_rewarded_step: try_key_as::<GenericList<_>>(values, "lastRewardedStep")?.into(),
            last_rewarded_score: try_key_as::<GenericList<_>>(values, "lastRewardedScore")?.into(),
            // onUpdated
            version: try_key_as(values, "version")?,
        })
    }
}

#[derive(Debug)]
struct GameModeId(i32);

impl TryFrom<&Value> for GameModeId {
    type Error = String;

    fn try_from(value: &Value) -> Result<Self, String> {
        from_id_object("GameModeId", value).map(Self)
    }
}

#[derive(Debug)]
struct QuestTileId(i32);

impl TryFrom<&Value> for QuestTileId {
    type Error = String;

    fn try_from(value: &Value) -> Result<Self, String> {
        from_id_object("QuestTileId", value).map(Self)
    }
}

// struct ActiveSessionQuestList(Vec<ActiveSessionQuest>);
//
// impl TryFrom<&Value> for ActiveSessionQuestList {
//     type Error = String;
//
//     fn try_from(value: &Value) -> Result<Self, String> {
//         let list = try_generic_list_from(value)?;
//         let list = list
//             .into_iter()
//             .map(|item| ActiveSessionQuest::try_from(item))
//             .collect::<Result<_, _>>()?;
//         Ok(Self(list))
//     }
// }

#[derive(Debug)]
struct QuestId(i32);

impl TryFrom<&Value> for QuestId {
    type Error = String;

    fn try_from(value: &Value) -> Result<Self, String> {
        from_id_object("Dorfromantik.QuestId", value).map(Self)
    }
}

#[derive(Debug)]
pub struct SegmentTypeId(pub i32);

impl TryFrom<&Value> for SegmentTypeId {
    type Error = String;

    fn try_from(value: &Value) -> Result<Self, String> {
        from_id_object("Dorfromantik.SegmentTypeId", value).map(Self)
    }
}

#[derive(Debug)]
pub struct SpecialTileId(pub i32);

impl TryFrom<&Value> for SpecialTileId {
    type Error = String;

    fn try_from(value: &Value) -> Result<Self, String> {
        from_id_object("Dorfromantik.SpecialTileId", value).map(Self)
    }
}
