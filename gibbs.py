import numpy as np
np.random.seed(12345)
from scipy.stats import norm, multivariate_normal
from statsmodels.tsa.stattools import acf
from model import simulate_st
import matplotlib.pyplot as plt
from tqdm import tqdm

# constraints for input values
CONSTRAINTS = [-0.4189, 0.4189, -3.2, 3.2, 7.319, 9.51, -5.0, 20.0]
# sampling time for motion model
DT = 0.1

if __name__ == '__main__':
    # data
    data = np.load('data/saved_traj.npz')
    all_traj = data['traj'] # N x 7 x 10
    all_input = data['input'] # N x 2 x 10

    # constants
    max_iter = 10000
    # [mu, C_Sf, C_Sr, lf, lr, h, m, I]
    theta_init_mean = np.array([1.0489, 4.718, 5.4562, 0.15875, 0.17145, 0.074, 3.74, 0.04712])
    theta_init_cov = 1.*np.eye(len(theta_init_mean))

    # init
    curr_theta = multivariate_normal.rvs(theta_init_mean, theta_init_cov, size=1)

    # book keeping
    theta_traj = np.empty((max_iter, len(theta_init_mean)))
    post_mean = np.empty((max_iter, len(theta_init_mean)))
    post_std = np.empty((max_iter, len(theta_init_mean)))
    theta_traj[0, :] = curr_theta
    post_mean[0, :] = curr_theta
    post_std[0, :] = np.ones(len(theta_init_mean))

    # outer gibbs loop
    for t in tqdm(range(1, max_iter)):
        # param loop
        for i in range(len(theta_init_mean)):
            all_err = []
            # data loop
            for j in range(all_traj.shape[0]):
                nominal_traj = simulate_st(all_traj[j, :, 0], DT, all_input[j, :, :].T, curr_theta, CONSTRAINTS)
                err = np.sqrt(((all_traj[j, :, :] - nominal_traj) ** 2)).mean()
                all_err.append(err)

            # construct likelihood
            all_err = np.array(all_err)
            lik_mean = np.mean(all_err)
            lik_std = np.std(all_err)
            prior_mean = theta_init_mean[i]
            prior_std = theta_init_cov[i, i]
            cond_mean = (lik_std**2 * prior_mean + prior_std**2 * lik_mean) / (lik_std**2 + prior_std**2)
            cond_std = 1 / ((1 / lik_std**2) + (1 / prior_std**2))
            # sample new theta_i from conditional
            curr_theta[i] = norm.rvs(cond_mean, np.sqrt(cond_std), size=1)
            # book keeping
            theta_traj[t, i] = curr_theta[i]
            post_mean[t, i] = cond_mean
            post_std[t, i] = cond_std


    np.savez_compressed('data/gibbs_traj_iter10000_thetafix.npz', theta_traj=theta_traj, max_iter=np.array([max_iter]), post_mean=post_mean, post_std=post_std)

    # chain thinning
    final_samp = theta_traj[::40, :]

    # histogram of final samples
    plt.hist(final_samp, 20, stacked=True)
    plt.legend(['mu', 'C_Sf', 'C_Sr', 'lf', 'lr', 'h', 'm', 'I'])
    plt.xlabel('Value')
    plt.ylabel('Count')
    plt.title('Histogram of Final Samples')
    plt.show()

    # history of final samples
    plt.plot(final_samp)
    plt.legend(['mu', 'C_Sf', 'C_Sr', 'lf', 'lr', 'h', 'm', 'I'])
    plt.xlabel('Iter')
    plt.ylabel('Value')
    plt.title('Final Samples of Parameters Theta')
    plt.show()

    # acf
    all_acf = []
    for d in range(final_samp.shape[1]):
        curr_acf = acf(final_samp[:, d])
        all_acf.append(curr_acf)
        plt.bar(range(curr_acf.shape[0]), curr_acf)

    plt.legend(['mu', 'C_Sf', 'C_Sr', 'lf', 'lr', 'h', 'm', 'I'])
    plt.title('acf plot of Final Samples')
    plt.show()
    all_acf = np.array(all_acf)

    # final sample mean
    final_post_mean = final_samp.mean(axis=0)