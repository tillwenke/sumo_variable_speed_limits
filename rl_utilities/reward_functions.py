def linear_occ_reward(x):
    if 9 < x < 13:
        return 2

    if 0 < x <= 11:
        return x/11
    elif 11 < x < 100:
        return (100-x)/89
    else:
        return 0

def quad_occ_reward(x):
    if 0 < x <= 12:
        return ((0.5 * x) + 6) / 12
    elif 12 < x < 80:
        return ((x-80)**2/68**2)
    else:
        return 0