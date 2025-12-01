import math

def calculate_r_min(v_cc, v_ol_max, i_ol):
    """
    Calculate the minimum I2C pull-up resistance.

    Arguments:
        v_cc: power supply voltage (V)
        v_ol_max: maximum low-level output voltage (V)
        i_ol: low-level output (sink) current (A)
    Returns:
        minimum pull-up resistance (ohms)
    """
    r = (v_cc - v_ol_max) / i_ol
    return r

def calculate_r_max(v_cc, v_il_max, v_ih_min, t_r, c):
    """
    Calculate the maximum I2C pull-up resistance.

    Arguments:
        v_cc: power supply voltage (V)
        v_il_max: maximum low-level input voltage (V)
        v_ih_min: minimum high-level input voltage (V)
        t_r: I2C rise time from maximum low-level input voltage to minimum high-level input voltage (s)
        c: bus capacitace for each I2C line (F)
    Returns:
        maximum pull-up resistance (ohms)
    """
    r = t_r / (c * math.log((v_cc - v_il_max) / (v_cc - v_ih_min)))
    return r
