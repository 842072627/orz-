import json
d = json.load(open(r'C:\Users\Administrator\Desktop\test-studio-mapf\big-final-test\game.domain\orz-example_4000.json'))
cols = d['cols']
rows = d['rows']
loc = 771709
r = loc // cols
c = loc % cols
print("loc=%d: row=%d col=%d (grid %dx%d)" % (loc, r, c, rows, cols))
print("map cell: %d (0=walkable)" % d['map'][loc])
for dr in range(-5, 6):
    for dc in range(-5, 6):
        nr, nc = r+dr, c+dc
        if 0 <= nr < rows and 0 <= nc < cols:
            nl = nr*cols+nc
            if d['map'][nl] == 0:
                print("  walkable: (%d,%d) at loc %d" % (nc, nr, nl))
                break
