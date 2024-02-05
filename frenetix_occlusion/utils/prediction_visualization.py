__author__ = "Maximilian Geisslinger, Tobias Markus"
__copyright__ = "TUM Professorship Autonomous Vehicle Systems"
__version__ = "1.0"
__maintainer__ = "Korbinian Moller"
__email__ = "korbinian.moller@tum.de"
__status__ = "Beta"

import sys
import numpy as np
from matplotlib.patches import Ellipse
import matplotlib.transforms as transforms


def confidence_ellipse(mu, cov, ax, n_std=3.0, facecolor="red", **kwargs):
    """
    Create a plot of the covariance confidence ellipse of *x* and *y*.

    Parameters
    ----------
    x, y : array-like, shape (n, )
        Input data.

    ax : matplotlib.axes.Axes
        The axes object to draw the ellipse into.

    n_std : float
        The number of standard deviations to determine the ellipse's radiuses.

    **kwargs
        Forwarded to `~matplotlib.patches.Ellipse`

    Returns
    -------
    matplotlib.patches.Ellipse
    """

    mu_x = mu[0]
    mu_y = mu[1]

    pearson = cov[0][1] / (np.sqrt(cov[0][0] * cov[1][1]) + sys.float_info.epsilon)
    # Using a special case to obtain the eigenvalues of this two-dimensional dataset.
    ell_radius_x = np.sqrt(1 + pearson)
    ell_radius_y = np.sqrt(1 - pearson)
    ellipse = Ellipse((0, 0), width=ell_radius_x * 2, height=ell_radius_y * 2, facecolor=facecolor, alpha=0.2,
                      zorder=50, **kwargs)

    # Calculating the standard deviation of x from the square root of the variance and multiplying
    # with the given number of standard deviations.
    scale_x = np.sqrt(cov[0][0]) * n_std

    # calculating the standard deviation of y ...
    scale_y = np.sqrt(cov[1][1]) * n_std

    transf = (
        transforms.Affine2D()
        .rotate_deg(45)
        .scale(scale_x, scale_y)
        .translate(mu_x, mu_y)
    )

    ellipse.set_transform(transf + ax.transData)
    return ax.add_patch(ellipse)


def draw_with_uncertainty(fut_pos_list, fut_cov_list, ax):

    for i, fut_pos in enumerate(fut_pos_list):
        for j, pos in enumerate(fut_pos):
            confidence_ellipse(
                pos, fut_cov_list[i][j], ax, n_std=1.0, facecolor="yellow"
            )
        for j, pos in enumerate(fut_pos):
            confidence_ellipse(
                pos, fut_cov_list[i][j], ax, n_std=0.5, facecolor="orange"
            )
        for j, pos in enumerate(fut_pos):
            confidence_ellipse(pos, fut_cov_list[i][j], ax, n_std=0.2, facecolor="red")


def draw_uncertain_predictions(prediction_dict, ax, draw_to=30):
    """Draw predictions and visualize uncertainties with heat maps.

    Args:
        prediction_dict ([dict]): [prediction dicts with key obstacle id and value pos_list and cov_list]
        ax ([type]): [matplotlib.ax to plot in]
        draw_to: [int]: [number of prediction steps]
    """

    prediction_plot_list = list(prediction_dict.values())[:10]
    fut_pos_list = [
        prediction_plot_list[i]["pos_list"][:draw_to][:]
        for i in range(len(prediction_plot_list))
    ]

    fut_cov_list = [
        prediction_plot_list[i]["cov_list"][:draw_to][:]
        for i in range(len(prediction_plot_list))
    ]
    draw_with_uncertainty(fut_pos_list, fut_cov_list, ax)

