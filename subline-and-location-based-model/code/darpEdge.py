from darpNode import DarpNode

class DarpEdge:
    def __init__(self, u: DarpNode, v: DarpNode) -> None:
        self._from = u
        self._to = v
        self.label = (self._from.name, self._to.name)
        self.distance = None

        self._set_direction()
    
    def _set_direction(self) -> None:
        if self._from.direction != self._to.direction:
            self.direction = "T"
        elif self._from.direction is not None:
            self.direction = self._from.direction
        elif self._to.direction is not None:
            self.direction = self._to.direction
        else:
            self.direction = None

    def __iter__(self):
        yield from (self._from, self._to)

    def __eq__(self, other):
        i,j = other
        return (i == self._from) and (j == self._to)