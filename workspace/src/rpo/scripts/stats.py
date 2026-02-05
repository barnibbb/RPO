import pandas as pd
import matplotlib.pyplot as plt


def make_boxplot(data, labels, title, ylabel,ylims):
    plt.figure()
    plt.boxplot(data,labels=labels,showfliers=False)
    plt.title(title)
    plt.ylabel(ylabel)
    plt.grid(True, axis="y")
    plt.ylim(ylims)
    plt.tight_layout()
    plt.show()



if __name__ == "__main__":

    # df = pd.read_csv("/home/appuser/data/z_coverage.csv")

    # datasets = sorted(df["dataset"].unique())

    # coverage_data = [df[df["dataset"] == d]["coverage"].values for d in datasets]
    # runtime_data = [df[df["dataset"] == d]["runtime"].values for d in datasets]

    # make_boxplot(coverage_data, datasets, "Optimized Coverage", "Coverage Ratio", ylims=(0,1))

    # make_boxplot(runtime_data, datasets, "Runtime Comparison", "Runtime [s]", ylims=(0,1800))

    # data = df.drop(columns=["offset"])
    # box_data = [data[col].values for col in data.columns]
    # labels = data.columns

    # make_boxplot(box_data, labels, "Z coverage", "Coverage Ratio", ylims=(0,100))

    # approaches = ["1","2","3","4","5","6","7","8","9"]
    # # Office
    # runtime = [0.12,22.76,35.00,52.31,41.96,701.22,186.28,168.97,9.27]
    # coverage = [32.27,77.93,81.13,82.11,83.27,84.96,84.78,85.03,86.75]
    # markers = ['o','s','^','v','D','P','*','X','<']

    # plt.figure()

    # for a, x, y, m in zip(approaches, runtime, coverage, markers):
    #     plt.scatter(x,y,marker=m,s=100,label=a)

    # plt.xlim(0, 100)
    # plt.ylim(0, 100)
    # plt.xlabel("Runtime")
    # plt.ylabel("Coverage")
    # plt.title("Coverage vs Runtime")
    # plt.legend(ncol=3)
    # plt.grid(True)
    # plt.show()
    
