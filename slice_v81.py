import json
d = json.load(open(r'C:\Users\Administrator\Desktop\test-studio-mapf\LORR26_842072627\v81_4000_2500.json'))
# Truncate actualPaths to first 500 timesteps
count = 0
for i in range(len(d.get('actualPaths',[]))):
    d['actualPaths'][i] = d['actualPaths'][i][:500]
d['makespan'] = 500
nt = 0
for t in d.get('tasks',[]):
    if t.get('end',99999) <= 500:
        nt += 1
d['numTaskFinished'] = nt
print("agents:", len(d.get('actualPaths',[])), "| tasks@500:", nt)
json.dump(d, open(r'C:\Users\Administrator\Desktop\test-studio-mapf\LORR26_842072627\v81_500_slice.json','w'))
