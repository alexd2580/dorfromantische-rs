# https://stackoverflow.com/questions/3052202/how-to-analyse-contents-of-binary-serialization-stream/30176566#30176566

from enum import Enum
from dataclasses import dataclass
from typing import Any, Optional
import struct
import sys


def read_int(file, n, signed=False):
    value = int.from_bytes(file.read(n), "little")
    if signed and value & 0b1 << 31:
        return value - 2 * (0b1 << 31)
    return value


def read_boolean(file):
    return read_int(file, 1) == 1


def read_string(file):
    # https://winprotocoldoc.blob.core.windows.net/productionwindowsarchives/MS-NRBF/%5bMS-NRBF%5d.pdf#%5B%7B%22num%22%3A66%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C69%2C670%2C0%5D
    length = 0
    num_bytes = 0
    while True:
        next_byte = read_int(file, 1)
        length += (next_byte & 0b01111111) << (num_bytes * 7)
        num_bytes += 1
        if (next_byte & 0b10000000) == 0:
            break

    content = file.read(length)
    return content.decode("utf-8")


def read_single(file):
    return struct.unpack("f", file.read(4))[0]


class ReadableEnum(Enum):
    @classmethod
    def read(cls, file):
        return cls(read_int(file, 1))


class BinaryType(ReadableEnum):
    Primitive = 0
    String = 1
    Object = 2
    SystemClass = 3
    Class = 4
    ObjectArray = 5
    StringArray = 6
    PrimitiveArray = 7


class PrimitiveType(ReadableEnum):
    Boolean = 1
    Byte = 2
    Char = 3
    Decimal = 5
    Double = 6
    Int16 = 7
    Int32 = 8
    Int64 = 9
    SByte = 10
    Single = 11
    TimeSpan = 12
    DateTime = 13
    UInt16 = 14
    UInt32 = 15
    UInt64 = 16
    Null = 17
    String = 18


class RecordType(ReadableEnum):
    SerializedStreamHeader = 0
    ClassWithId = 1
    SystemClassWithMembers = 2
    ClassWithMembers = 3
    SystemClassWithMembersAndTypes = 4
    ClassWithMembersAndTypes = 5
    BinaryObjectString = 6
    BinaryArray = 7
    MemberPrimitiveTyped = 8
    MemberReference = 9
    ObjectNull = 10
    MessageEnd = 11
    BinaryLibrary = 12
    ObjectNullMultiple256 = 13
    ObjectNullMultiple = 14
    ArraySinglePrimitive = 15
    ArraySingleObject = 16
    ArraySingleString = 17
    MethodCall = 21
    MethodReturn = 22


class BinaryArrayType(ReadableEnum):
    # A single-dimensional Array.
    Single = 0
    # An Array whose elements are Arrays. The elements of a jagged Array can be of different dimensions and sizes.
    Jagged = 1
    # A multi-dimensional rectangular Array.
    Rectangular = 2
    # A single-dimensional offset.
    SingleOffset = 3
    # A jagged Array where the lower bound index is greater than 0.
    JaggedOffset = 4
    # Multi-dimensional Arrays where the lower bound index of at least one of the dimensions is greater than 0.
    RectangularOffset = 5


def read_primitive(file, primitive_type: PrimitiveType):
    match primitive_type:
        case PrimitiveType.Boolean:
            return read_boolean(file)
        case PrimitiveType.Byte:
            return read_int(file, 1)
        # case PrimitiveType.Char:
        # case PrimitiveType.Decimal:
        # case PrimitiveType.Double:
        case PrimitiveType.Int16:
            return read_int(file, 2, signed=True)
        case PrimitiveType.Int32:
            return read_int(file, 4, signed=True)
        case PrimitiveType.Int64:
            return read_int(file, 8, signed=True)
        # case PrimitiveType.SByte :
        case PrimitiveType.Single :
            return read_single(file)
        # case PrimitiveType.TimeSpan :
        # case PrimitiveType.DateTime:
        case PrimitiveType.UInt16:
            return read_int(file, 2)
        case PrimitiveType.UInt32:
            return read_int(file, 4)
        case PrimitiveType.UInt64:
            return read_int(file, 8)
        # case PrimitiveType.Null:
        case PrimitiveType.String:
            return read_string(file)
        case _:
            raise NotImplementedError(f"No deserializer defined for {primitive_type}")


@dataclass
class ClassInfo:
    object_id: int
    name: str
    member_names: list[str]

    @staticmethod
    def read(file):
        object_id = read_int(file, 4)
        name = read_string(file)
        member_count = read_int(file, 4)
        member_names = [read_string(file) for _ in range(member_count)]
        return ClassInfo(object_id, name, member_names)


@dataclass
class ClassTypeInfo:
    type_name: str
    library_id: int

    @staticmethod
    def read(file):
        type_name = read_string(file)
        library_id = read_int(file, 4)
        return ClassTypeInfo(type_name, library_id)


def read_additional_info(file, binary_type: BinaryType):
    match binary_type:
        case BinaryType.Primitive:
            return PrimitiveType.read(file)
        case BinaryType.SystemClass:
            return read_string(file)
        case BinaryType.Class:
            return ClassTypeInfo.read(file)
        case BinaryType.PrimitiveArray:
            return PrimitiveType.read(file)
        case _:
            return None


@dataclass
class MemberTypeInfo:
    binary_types: list[BinaryType]
    additional_infos: list[Any]

    @staticmethod
    def read(file, n):
        binary_types = [BinaryType.read(file) for _ in range(n)]
        additional_infos = [read_additional_info(file, binary_type) for binary_type in binary_types]
        return MemberTypeInfo(binary_types, additional_infos)


@dataclass
class ArrayInfo:
    object_id: int
    length: int

    @staticmethod
    def read(file):
        object_id = read_int(file, 4)
        length = read_int(file, 4)
        return ArrayInfo(object_id, length)


@dataclass
class SerializationHeaderRecord:
    root_id: int
    header_id: int
    major_version: int
    minor_version: int

    @staticmethod
    def read(file, classes):
        root_id = read_int(file, 4)
        header_id = read_int(file, 4)
        major_version = read_int(file, 4)
        minor_version = read_int(file, 4)
        return SerializationHeaderRecord(root_id, header_id, major_version, minor_version)


@dataclass
class ClassWithId:
    object_id: int
    class_info: ClassInfo
    values: list[Any]

    @staticmethod
    def read(file, classes):
        object_id = read_int(file, 4)
        class_id = read_int(file, 4)
        class_info, member_type_info = classes[class_id]
        values = ClassWithMembersAndTypes.read_values(file, classes, class_info, member_type_info)
        return ClassWithId(object_id, class_info, values)


@dataclass
class ClassWithMembers:
    """Class with inferrable member representations."""
    class_info: ClassInfo
    library_id: int
    values: list[Any]

    @staticmethod
    def read(file, classes):
        class_info = ClassInfo.read(file)
        # Sure??
        classes[class_info.object_id] = class_info, None # ASKJDHAKSJDH
        library_id = read_int(file, 4)
        values = [read_record(file, classes) for _ in range(len(class_info.member_names))]
        return ClassWithMembers(class_info, library_id, values)


class SystemClassWithMembersAndTypes:
    """System-level class without 'namespace'."""
    @staticmethod
    def read(file, classes):
        class_info = ClassInfo.read(file)
        member_type_info = MemberTypeInfo.read(file, len(class_info.member_names))
        # Sure??
        classes[class_info.object_id] = class_info, member_type_info
        library_id = -1
        values = ClassWithMembersAndTypes.read_values(file, classes, class_info, member_type_info)
        return ClassWithMembersAndTypes(class_info, member_type_info, library_id, values)


@dataclass
class ClassWithMembersAndTypes:
    class_info: ClassInfo
    member_type_info: MemberTypeInfo
    library_id: int
    values: list[Any]

    @staticmethod
    def read_value(file, classes, member_name, binary_type, additional_infos):
        match binary_type:
            case BinaryType.Primitive:
                return read_primitive(file, additional_infos)
            case BinaryType.String:
                return read_record(file, classes)
            case BinaryType.SystemClass:
                return read_record(file, classes)
            case BinaryType.Class:
                return read_record(file, classes)
            case BinaryType.PrimitiveArray:
                return read_record(file, classes)
            case _:
                raise NotImplementedError(f"Don't know how to read member value {member_name}: {binary_type}")

    @staticmethod
    def read_values(file, classes, class_info, member_type_info):
        return [
            ClassWithMembersAndTypes.read_value(file, classes, member_name, binary_type, additional_infos)
            for member_name, binary_type, additional_infos
            in zip(class_info.member_names, member_type_info.binary_types, member_type_info.additional_infos)
        ]

    @staticmethod
    def read(file, classes):
        class_info = ClassInfo.read(file)
        member_type_info = MemberTypeInfo.read(file, len(class_info.member_names))
        classes[class_info.object_id] = class_info, member_type_info
        library_id = read_int(file, 4)
        values = ClassWithMembersAndTypes.read_values(file, classes, class_info, member_type_info)
        return ClassWithMembersAndTypes(class_info, member_type_info, library_id, values)


@dataclass
class BinaryObjectString:
    object_id: int
    value: str

    @staticmethod
    def read(file, classes):
        object_id = read_int(file, 4)
        value = read_string(file)
        return BinaryObjectString(object_id, value)


@dataclass
class BinaryArray:
    object_id: int
    array_type: BinaryArrayType
    rank: int
    lengths: list[int]
    lower_bounds: Optional[list[int]]
    item_type: BinaryType
    additional_info: Any
    values: list[list[Any]]

    @staticmethod
    def read(file, classes):
        object_id = read_int(file, 4)
        array_type = BinaryArrayType.read(file)
        rank = read_int(file, 4)
        lengths = [read_int(file, 4) for _ in range(rank)]
        if array_type in [BinaryArrayType.SingleOffset, BinaryArrayType.JaggedOffset, BinaryArrayType.RectangularOffset]:
            lower_bounds = [read_int(file, 4) for _ in range(rank)]
        else:
            lower_bounds = None
        item_type = BinaryType.read(file)
        additional_info = read_additional_info(file, item_type)

        values = []
        for length in lengths:
            column = []
            while len(column) != length:
                record = read_record(file, classes)
                if isinstance(record, ObjectNullMultiple):
                    column.extend([None for _ in range(record.length)])
                else:
                    column.append(record)
            values.append(column)

        return BinaryArray(object_id, array_type, rank, lengths, lower_bounds, item_type, additional_info, values)


@dataclass
class MemberReference:
    id_ref: int

    @staticmethod
    def read(file, classes):
        id_ref = read_int(file, 4)
        return MemberReference(id_ref)


@dataclass
class ObjectNull:
    @staticmethod
    def read(file, classes):
        return ObjectNull()


@dataclass
class BinaryLibrary:
    library_id: int
    library_name: str

    @staticmethod
    def read(file, classes):
        library_id = read_int(file, 4)
        library_name = read_string(file)
        return BinaryLibrary(library_id, library_name)


class ObjectNullMultiple256:
    @staticmethod
    def read(file, classes):
        length = read_int(file, 1)
        return ObjectNullMultiple(length)


@dataclass
class ObjectNullMultiple:
    length: int

    @staticmethod
    def read(file, classes):
        length = read_int(file, 4)
        return ObjectNullMultiple(length)


@dataclass
class ArraySinglePrimitive:
    array_info: ArrayInfo
    primitive_type: PrimitiveType
    values: list[Any]

    @staticmethod
    def read(file, classes):
        array_info = ArrayInfo.read(file)
        primitive_type = PrimitiveType.read(file)
        values = [read_primitive(file, primitive_type) for _ in range(array_info.length)]
        return ArraySinglePrimitive(array_info, primitive_type, values)


def read_record(file, classes):
    RECORD_CLASSES = {
        RecordType.SerializedStreamHeader: SerializationHeaderRecord,
        RecordType.ClassWithId: ClassWithId,
        # RecordType.SystemClassWithMembers: None,
        RecordType.ClassWithMembers: ClassWithMembers,
        RecordType.SystemClassWithMembersAndTypes: SystemClassWithMembersAndTypes,
        RecordType.ClassWithMembersAndTypes: ClassWithMembersAndTypes,
        RecordType.BinaryObjectString: BinaryObjectString,
        RecordType.BinaryArray: BinaryArray,
        # RecordType.MemberPrimitiveTyped: None,
        RecordType.MemberReference: MemberReference,
        RecordType.ObjectNull: ObjectNull,
        # RecordType.MessageEnd: None,
        RecordType.BinaryLibrary: BinaryLibrary,
        RecordType.ObjectNullMultiple256: ObjectNullMultiple256,
        RecordType.ObjectNullMultiple: ObjectNullMultiple,
        RecordType.ArraySinglePrimitive: ArraySinglePrimitive,
        # RecordType.ArraySingleObject: None,
        # RecordType.ArraySingleString: None,
        # RecordType.MethodCall: None,
        # RecordType.MethodReturn: None,
    }

    next_record_type = RecordType.read(file)
    if next_record_type == RecordType.MessageEnd:
        return

    record_class = RECORD_CLASSES[next_record_type]
    return record_class.read(file, classes)


def simplify_values(values):
    for record in values:
        simplified = simplify_record(record)
        yield simplified if not isinstance(simplified, tuple) else simplified[1]


def simplify_record(record):
    if isinstance(record, SerializationHeaderRecord):
        return "root", MemberReference(record.root_id)
    if isinstance(record, ClassWithId):
        class_info = record.class_info
        name = class_info.name
        data = dict(zip(class_info.member_names, simplify_values(record.values))) | {"__class": name}
        if "Generic.List" in name and "_items" in data:
            data = data["_items"]
        return record.object_id, data
    if isinstance(record, ClassWithMembers):
        class_info = record.class_info
        name = class_info.name
        data = dict(zip(class_info.member_names, simplify_values(record.values))) | {"__class": name}
        if "Generic.List" in name and "_items" in data:
            data = data["_items"]
        return record.class_info.object_id, data
    if isinstance(record, ClassWithMembersAndTypes):
        class_info = record.class_info
        name = class_info.name
        data = dict(zip(class_info.member_names, simplify_values(record.values))) | {"__class": name}
        if "Generic.List" in name and "_items" in data:
            data = data["_items"]
        return record.class_info.object_id, data
    if isinstance(record, BinaryObjectString):
        return record.object_id, record.value
    if isinstance(record, BinaryArray):
        assert record.array_type == BinaryArrayType.Single
        return record.object_id, list(simplify_values(record.values[0]))
    if isinstance(record, MemberReference):
        return None, record
    if isinstance(record, ObjectNull):
        return None, None
    if isinstance(record, ObjectNullMultiple):
        return None, [None for _ in range(record.length)]
    if isinstance(record, ArraySinglePrimitive):
        if record.primitive_type == PrimitiveType.Byte:
            value = bytes(record.values)
        else:
            value = list(simplify_values(record.values))
        return record.array_info.object_id, value
        # return record.array_info.object_id, value

    return record


def resolve_references(record, data):
    """Recursively replace all references in `record` with their actual data."""
    if isinstance(record, MemberReference):
        return resolve_references(data[record.id_ref], data)
    if isinstance(record, list):
        return [resolve_references(x, data) for x in record]
    if isinstance(record, dict):
        return {k: resolve_references(v, data) for (k, v) in record.items()}

    return record



def pythonify(records):
    simplified = [simplify_record(record) for record in records]
    simple_records = dict(filter(lambda x: isinstance(x, tuple), simplified))
    return resolve_references(simple_records["root"], simple_records)

def parse_savegame(args):
    classes = {}
    records = []

    with open(args[0], "rb") as file:
        while record := read_record(file, classes):
            records.append(record)

    return pythonify(records)


if __name__ == "__main__":
    from pprint import pprint

    pprint(parse_savegame(sys.argv[1:]))
