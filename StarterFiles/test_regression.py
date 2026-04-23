
def ls_regression(x: list[int], y: list[float]):
    """
    Calculates the least squares regression slope.
    """
    sum_x = sum(x)
    sum_y = sum(y)
    sum_x_2 = sum([i**2 for i in x])
    sum_xy = sum([i*j for i,j in zip(x,y)])
    N = len(x)

    return (N * sum_xy - sum_x * sum_y) / (N * sum_x_2 - sum_x**2)


if __name__ == "__main__":
    x = [2,3,5,7,9]
    y = [4,5,7,10,15]

    print(ls_regression(x,y))
