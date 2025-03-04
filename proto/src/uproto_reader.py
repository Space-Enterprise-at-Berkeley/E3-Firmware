import io
import re
import json


class JSONCDecoder(json.JSONDecoder):

    _comment_pattern = re.compile(r"(\/\/.*?\n|\/\*.*?\*\/)", re.DOTALL)

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def decode(self, source):
        filtered = io.StringIO()
        index = 0
        for match in JSONCDecoder._comment_pattern.finditer(source):
            s, e = match.start(1), match.end(1)
            filtered.write(source[index:s])
            index = e
        filtered.write(source[index:])
        return super().decode(filtered.getvalue())
