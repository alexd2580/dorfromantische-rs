from collections import namedtuple
import math


ChunkHeader = namedtuple("ChunkHeader", "s t")


class Chunks:
    chunk_size: int
    num_chunk_levels: int
    num_level0_chunks: int

    headers: list[list[ChunkHeader]]
    indices: list[list[int]]

    def _gen_chunks(self, items: list[tuple[int, int, int]], chunk_width: int, chunk_size: int) -> tuple[list[ChunkHeader], list[int]]:
        """Generate chunks for `items`.

        `width` gives the coordinate bb width.
        `size` defines the number of items along the s and t axes.
        """
        chunks = {}
        for index, s, t in items:
            # Position of the chunk.
            chunk_s = int(math.floor(s / chunk_width)) * chunk_width
            chunk_t = int(math.floor(t / chunk_width)) * chunk_width
            chunk_pos = ChunkHeader(chunk_s, chunk_t)
            chunk = chunks.get(chunk_pos)
            if not chunk:
                chunk = [-1] * chunk_size ** 2

            # Position of the item.
            items_per_step = int(chunk_width / chunk_size)
            local_s = int((s - chunk_s) / items_per_step)
            local_t = int((t - chunk_t) / items_per_step)
            local_index = local_t * chunk_size + local_s

            chunk[local_index] = index
            chunks[chunk_pos] = chunk

        return list(chunks.keys()), list(chunks.values())



    def __init__(self, items: list[tuple[int, int, int]], num_chunk_levels: int = 3, chunk_size: int = 2):
        if num_chunk_levels > 8:
            raise NotImplementedError(f"Can't have more than 8 chunk_levels, you specified {num_chunk_levels}")


        self.chunk_size = chunk_size
        self.num_chunk_levels = num_chunk_levels

        header_groups = []
        indice_groups = []

        for level in range(num_chunk_levels):
            width = chunk_size ** (level + 1)
            headers, indices = self._gen_chunks(items, width, chunk_size)

            header_groups.append(headers)
            indice_groups.append(indices)

            self.num_level0_chunks = len(headers)

            items = [(index, s, t) for index, (s, t) in enumerate(headers)]

        self.headers = []
        self.indices = []

        offset = 0

        # Go through levels backwards.
        for level in range(num_chunk_levels - 1, 0, -1):
            offset += len(header_groups[level])

            self.headers.extend(header_groups[level])
            for index_group in indice_groups[level]:
                self.indices.append([-1 if i == -1 else offset + i for i in index_group])


        self.headers.extend(header_groups[0])
        self.indices.extend(indice_groups[0])
