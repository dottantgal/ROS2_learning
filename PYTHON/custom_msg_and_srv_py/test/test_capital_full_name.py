import pytest

from custom_msg_and_srv_py.formatting import build_capital_full_name, normalize_token


@pytest.mark.parametrize(
    ('raw', 'expected'),
    [
        ('alice', 'ALICE'),
        (' Bob ', 'BOB'),
        ('', ''),
        (None, ''),
    ],
)
def test_normalize_token(raw, expected):
    assert normalize_token(raw) == expected


@pytest.mark.parametrize(
    ('name', 'surname', 'expected'),
    [
        ('alice', 'smith', 'ALICE SMITH'),
        ('ALICE', 'SMITH', 'ALICE SMITH'),
        ('', 'smith', 'SMITH'),
        ('alice', '', 'ALICE'),
        (' ', ' ', ''),
    ],
)
def test_build_capital_full_name(name, surname, expected):
    _, _, capital_full_name = build_capital_full_name(name, surname)
    assert capital_full_name == expected
