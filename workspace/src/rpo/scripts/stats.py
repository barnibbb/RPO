import pandas as pd
import matplotlib.pyplot as plt


def make_boxplot(data, labels, title, ylabel,ylims):
    plt.figure()
    plt.boxplot(data,labels=labels,showfliers=True)
    plt.title(title)
    plt.ylabel(ylabel)
    plt.grid(True, axis="y")
    plt.ylim(ylims)
    plt.tight_layout()
    plt.show()



if __name__ == "__main__":

    df = pd.read_csv("/home/appuser/data/data.csv")

    datasets = sorted(df["dataset"].unique())

    coverage_data = [df[df["dataset"] == d]["coverage"].values for d in datasets]
    runtime_data = [df[df["dataset"] == d]["runtime"].values for d in datasets]

    make_boxplot(coverage_data, datasets, "Optimized Coverage", "Coverage Ratio", ylims=(0,1))

    make_boxplot(runtime_data, datasets, "Runtime Comparison", "Runtime [s]", ylims=(0,1800))
