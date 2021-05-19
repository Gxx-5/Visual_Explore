import numpy as np
import matplotlib.pyplot as plt


def get_kde(x, data_array, bandwidth=0.1):
    def gauss(x):
        import math
        return (1 / math.sqrt(2 * math.pi)) * math.exp(-0.5 * (x ** 2))

    N = len(data_array)
    res = 0
    if len(data_array) == 0:
        return 0
    for i in range(len(data_array)):
        res += gauss((x - data_array[i]) / bandwidth)
    res /= (N * bandwidth)
    return res


def func(input_array):
    input_array.sort()
    bandwidth = 1.05 * np.std(input_array) * (len(input_array) ** (-1 / 5))
    x_array = np.linspace(min(input_array), max(input_array), 50).tolist()

    # y_array = [get_kde(x_array[i], input_array, bandwidth) for i in range(x_array.shape[0])]
    y_array = []
    pre = 0
    peak = [-1, -1]
    for i in range(len(x_array)):
        y = get_kde(x_array[i], input_array, bandwidth)
        if peak[1] < 0 and y < pre:
            peak[0] = x_array[i - 1]
            peak[1] = pre
        pre = y
        y_array.append(y)

    # print(x_array)
    # print(y_array)
    # print(peak[0], np.mean(x_array), np.std(x_array))
    if np.std(x_array) < 10:
        factor = (10 - np.std(x_array)) / 10
        peak[0] = peak[0] * (1 - factor) + factor * np.mean(x_array)
    return peak[0], np.mean(x_array), np.std(x_array)
    # plt.figure(1)
    # plt.hist(input_array, bins=40, density=True, zorder=0)
    # plt.plot(x_array, y_array, color='red', linestyle='-', zorder=5)
    # plt.scatter(peak[0], peak[1], color="green", s=50, zorder=10)
    # plt.show()


# bias = 0.9
# input_array = np.random.randn(200).tolist()
# input_array = (np.random.randn(100)-0.1).tolist() + (np.random.randn(100) + 0.1).tolist()
# input_array = (np.random.randn(100) - bias).tolist() + (np.random.randn(100) + bias).tolist()

bias_list = []
peak_list = []
mean_list = []
std_list = []
# for bias in range(-9, 9, 1):
bias = -19
while bias < 19:
    input_array = (np.random.randn(100) + 10 - bias).tolist() + (np.random.randn(100) + 10 + bias).tolist()
    peak, mean, std = func(input_array)
    bias_list.append(bias)
    peak_list.append(peak)
    mean_list.append(mean)
    std_list.append(std)
    bias += 0.1

plt.figure(figsize=(15, 5))
plt.subplot(131)
plt.plot(bias_list, peak_list, color='red', linestyle='-', zorder=5)
plt.subplot(132)
plt.plot(bias_list, mean_list, color='red', linestyle='-', zorder=5)
plt.subplot(133)
plt.plot(bias_list, std_list, color='red', linestyle='-', zorder=5)
plt.show()
