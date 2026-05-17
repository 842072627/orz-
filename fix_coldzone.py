import sys

with open(sys.argv[1], 'r', encoding='utf-8') as f:
    content = f.read()

old = """                    int a_m, a_b;
                    Centerline::project(a_loc, a_m, a_b);
                    if (a_m >= 0) {
                        for (auto& loc : it->second.locations) {
                            int tm, tb;
                            Centerline::project(loc, tm, tb);
                            if (tm >= 0 && ((dir == +1 && tm > a_m) || (dir == -1 && tm < a_m))) {
                                has_forward_task = true;
                                break;
                            }
                        }
                    }"""

new = """                    int a_m, a_b;
                    Centerline::project(a_loc, a_m, a_b);
                    if (a_m >= 0) {
                        for (auto& loc : it->second.locations) {
                            int tm, tb;
                            Centerline::project(loc, tm, tb);
                            if (tm >= 0 && ((dir == +1 && tm > a_m) || (dir == -1 && tm < a_m))) {
                                has_forward_task = true;
                                break;
                            }
                        }
                    } else {
                        // off-centerline: can't determine direction, assume forward
                        has_forward_task = true;
                    }"""

if old in content:
    content = content.replace(old, new, 1)
    with open(sys.argv[1], 'w', encoding='utf-8') as f:
        f.write(content)
    print('Fixed')
else:
    print('Pattern not found')
    # debug
    idx = content.find('project(a_loc, a_m')
    print(f'Found at {idx}')
    if idx >= 0:
        print(repr(content[idx-30:idx+200]))
