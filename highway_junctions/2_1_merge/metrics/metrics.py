from matplotlib import pyplot as plt

data = {
    'baseline': {},
    'mcs'     : {},
    'mtfc'    : {},
    'mcs2'    : {},
    'rl'      : {},
}

# Read metrics data from files.
for approach in data.keys():
    file_name = f'{approach}_metrics.csv'

    with open(file_name, 'r') as file:
        data[approach]['mean_speed'] = [ float(val) for val in file.readline().split(',') ]
        data[approach]['flow'] = [ float(val) for val in file.readline().split(',') ]
        data[approach]['emissions'] = [ float(val) for val in file.readline().split(',') ]

metrics = ['mean_speed', 'flow', 'emissions']
titles = ['Mean Speed', 'Flow', 'Emissions']
colors = ['red', 'blue', 'orange', 'green', 'pink']

for title, metric in zip(titles, metrics):

    fig, ax = plt.subplots(1, figsize=(8,5))
    for approach, color in zip(data.keys(), colors):
        x = range(len(data[approach][metric]))
        y = data[approach][metric]

        plt.plot(x, y, color=color, alpha=0.5, label=approach)

        if metric == 'mean_speed':
            plt.axhline(y=min(y), color=color, alpha=0.5, linestyle = '--', linewidth=0.8)

    plt.title(title)
    plt.legend(loc="upper left")
    plt.savefig(f'./{metric}_plot.png')


