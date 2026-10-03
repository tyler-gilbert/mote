"""
Checkout the spaces sdk/package repos
"""

load("//@star/prelude/rules/sdk.star", "sdk_add_repo")

sdk_add_repo(
    "@star/packages",
    url = "https://github.com/work-spaces/packages",
    rev = "add-package-tracking",
)
