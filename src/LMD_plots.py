# This function just consolidates all the different plotting functions desired fro the IMU localization
# Lisa Dischinger

from matplotlib import pyplot as plt


def check_len(list_a, list_b):
    # this is used to just find the difference between list lengths for plotting ease
    # it is assumed that list_a will always be the larger of the two
    if len(list_a) != len(list_b):
        diff = len(list_a) - len(list_b)
    else:
        diff = 0
    return diff


def lmd_plot(x, y, x_label, y_label, color, now_date, save_ttl):
    # this function just plots the basics and includes labels and such
    if color == "red":
        c = 'r'
    elif color == "blue":
        c = 'b'
    else:
        c = 'g'
    plt.plot(x, y, c)
    plt.ylabel(y_label)
    plt.xlabel(x_label)
    plt.title(now_date)
    plt.savefig(str(now_date) + " " + save_ttl + ".png")
    plt.show()


def lmd_three_subplot(time, plot_a, plot_b, plot_c, x_label, y_labels, title, save_plot, path):
    # correct for potential mismatch of list lengths
    diff_a = -1 * check_len(time, plot_a)
    diff_b = -1 * check_len(time, plot_b)
    diff_c = -1 * check_len(time, plot_c)

    # share x only
    ax1 = plt.subplot(311)
    if diff_a:
        plt.plot(time[:diff_a], plot_a)
    else:
        plt.plot(time, plot_a)
    plt.ylabel(y_labels[0])
    plt.title(title)
    plt.setp(ax1.get_xticklabels(), fontsize=2)

    ax2 = plt.subplot(312, sharex=ax1)
    if diff_b:
        plt.plot(time[:diff_b], plot_b)
    else:
        plt.plot(time, plot_b)
    plt.ylabel(y_labels[1])
    plt.setp(ax2.get_xticklabels(), visible=False)              # make these tick labels invisible

    ax3 = plt.subplot(313, sharex=ax1)
    if diff_c:
        plt.plot(time[:diff_c], plot_c)
    else:
        plt.plot(time, plot_c)
    plt.ylabel(y_labels[2])
    plt.xlabel(x_label)

    if save_plot:
        plt.savefig(path + title + ".png")
    plt.show()


def lmd_two_subplot(time, plot_a, plot_b, x_label, ya_label, yb_label, now_date, save_ttl, save_plot):
    # correct for potential mismatch of list lengths
    diff_a = -1*check_len(time, plot_a)
    diff_b = -1*check_len(time, plot_b)

    # share x only
    ax1 = plt.subplot(211)
    sub = time[:diff_b]
    if diff_a:
        plt.plot(time[:diff_a], plot_a)
    else:
        plt.plot(time, plot_a)
    plt.ylabel(ya_label)
    plt.title(now_date)
    plt.setp(ax1.get_xticklabels(), fontsize=2)

    ax2 = plt.subplot(212, sharex=ax1)
    if diff_b:
        plt.plot(time[:diff_b], plot_b)
    else:
        plt.plot(time, plot_b)
    plt.ylabel(yb_label)
    plt.xlabel(x_label)

    if save_plot:
        plt.savefig(str(now_date) + " " + save_ttl + ".png")
    plt.show()


def lmd_overlay2_subplot(plot_a, plot_a2, plot_b, plot_b2, plot_c, plot_c2,
                        x_label, ya_label, yb_label, yc_label, labels, now_date, save_ttl, plt_title):

    a = labels[0]
    b = labels[1]

    # share x only
    ax1 = plt.subplot(311)
    plt.plot(plot_a[:, 0], plot_a[:, 1], 'r', label=a)
    plt.plot(plot_a2[:, 0], plot_a2[:, 1], 'b', label=b)
    plt.ylabel(ya_label)
    plt.title(plt_title)
    plt.legend()
    plt.setp(ax1.get_xticklabels(), fontsize=2)

    ax2 = plt.subplot(312, sharex=ax1)
    plt.plot(plot_b[:, 0], plot_b[:, 1], 'r', label=a)
    plt.plot(plot_b2[:, 0], plot_b2[:, 1], 'b', label=b)
    plt.ylabel(yb_label)
    # plt.legend()
    plt.setp(ax2.get_xticklabels(), visible=True)              # make these tick labels invisible

    ax3 = plt.subplot(313, sharex=ax1)
    plt.plot(plot_c[:, 0], plot_c[:, 1], 'r', label=a)
    plt.plot(plot_c2[:, 0], plot_c2[:, 1], 'b', label=b)
    plt.ylabel(yc_label)
    plt.xlabel(x_label)
    # plt.legend()

    title = '/home/disch/catkin_ws/src/path_tracker/Data' + save_ttl + " "+ str(now_date) + ".png"
    plt.savefig(title)
    plt.show()

    # print("gathering input??")
    # input('Press Enter when you are done interacting with the plots')


def lmd_overlay3_subplot(plot_a, plot_a2, plot_a3, plot_b, plot_b2, plot_b3, plot_c, plot_c2, plot_c3,
                        x_label, ya_label, yb_label, yc_label, labels, now_date, save_ttl):

    a = labels[0]
    b = labels[1]
    c = labels[2]
    # share x only
    ax1 = plt.subplot(311)
    plt.plot(plot_a[:, 0], plot_a[:, 1], 'r', label=a)
    plt.plot(plot_a2[:, 0], plot_a2[:, 1], 'b', label=b)
    plt.plot(plot_a3[:, 0], plot_a3[:, 1], 'g', label=c)
    plt.ylabel(ya_label)
    plt.title(now_date)
    plt.legend()
    plt.setp(ax1.get_xticklabels(), fontsize=2)

    ax2 = plt.subplot(312, sharex=ax1)
    plt.plot(plot_b[:, 0], plot_b[:, 1], 'r', label=a)
    plt.plot(plot_b2[:, 0], plot_b2[:, 1], 'b', label=b)
    plt.plot(plot_b3[:, 0], plot_b3[:, 1], 'g', label=c)
    plt.ylabel(yb_label)
    plt.legend()
    plt.setp(ax2.get_xticklabels(), visible=True)              # make these tick labels invisible

    ax3 = plt.subplot(313, sharex=ax1)
    plt.plot(plot_c[:, 0], plot_c[:, 1], 'r', label=a)
    plt.plot(plot_c2[:, 0], plot_c2[:, 1], 'b', label=b)
    plt.plot(plot_c3[:, 0], plot_c3[:, 1], 'g', label=c)
    plt.ylabel(yc_label)
    plt.xlabel(x_label)
    plt.legend()

    plt.savefig(str(now_date) + " " + save_ttl + ".png")
    plt.show()