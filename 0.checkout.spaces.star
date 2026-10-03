"""
Checkout the spaces sdk/package repos
"""

load(
    "//@star/prelude/rules/checkout.star",
    "checkout_add_repo",
)

checkout_add_repo(
    "@star/sdk",
    url = "https://github.com/work-spaces/sdk",
    rev = "v0.5.1",
)

checkout_add_repo(
    "@star/packages",
    url = "https://github.com/work-spaces/packages",
    rev = "v0.2.73",
)
