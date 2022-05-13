from joblib import load, dump
import os

dir_path = os.path.dirname(os.path.realpath(__file__))

r1 = load(dir_path + '/results1.gz')
r3 = load(dir_path + '/results3.gz')
r4 = load(dir_path + '/results4.gz')

r = r1 + r3 + r4

new_r = []

for world in ['open world', 'closed world']:
    for agent in ['random', 'full agent (with flocking etc)']:
        for config in [1, 3, 4]:
            index = int(input('enter index for configuration %d; %s world; agent %s: ' % (config, world, agent)))
            if agent == 'random':
                index += len(r1)
            else:
                if config == 3:
                    index += len(r1)
                elif config == 4:
                    index += len(r1) + len(r3)
            data = r[index - 1]
            new_r.append(data[1:])

dump(new_r, 'all_results.gz')