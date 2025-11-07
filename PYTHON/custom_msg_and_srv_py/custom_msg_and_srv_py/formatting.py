from typing import Tuple


def normalize_token(value: str) -> str:
    """Trim and uppercase a name token."""
    if value is None:
        return ''
    return value.strip().upper()


def build_capital_full_name(name: str, surname: str) -> Tuple[str, str, str]:
    """Return normalized tokens and the combined capitalized full name string."""
    normalized_name = normalize_token(name)
    normalized_surname = normalize_token(surname)
    tokens = [token for token in (normalized_name, normalized_surname) if token]
    capital_full_name = ' '.join(tokens)
    return normalized_name, normalized_surname, capital_full_name
