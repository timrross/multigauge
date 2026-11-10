import re
TOK = re.compile(r'\(|\)|"(?:[^"\\]|\\.)*"|[^\s()]+')
def parse(text):
    stack = [[]]
    for m in TOK.finditer(text):
        t = m.group(0)
        if t == '(':
            new = []; stack[-1].append(new); stack.append(new)
        elif t == ')': stack.pop()
        elif t[0] == '"': stack[-1].append(t[1:-1].replace('\\"', '"').replace('\\\\', '\\'))
        else: stack[-1].append(t)
    return stack[0][0]
def find(node, key): return [c for c in node if isinstance(c, list) and c and c[0] == key]
def get(node, key, default=None):
    r = find(node, key)
    return (r[0][1] if len(r[0]) > 1 else '') if r else default
