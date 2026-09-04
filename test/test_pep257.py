# Copyright 2015 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_pep257.main import main
import pytest


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    # Same policy as .flake8, which ament_pep257 does not read.
    #
    # D1xx  these are scripts and internal helpers, not a published API; every
    #       module and non-obvious function is documented, and docs/DESIGN.md
    #       carries the reasoning that matters.
    # D212/D213  summary placement. This project puts it on the first line.
    # D401  imperative mood; several docstrings name what is returned instead.
    # scripts/dev  retired scaffolding, kept for the record rather than linted.
    rc = main(argv=[
        '.', 'test',
        '--exclude', 'scripts/dev',
        '--add-ignore', 'D100,D101,D102,D103,D104,D105,D106,D107,D212,D213,D401',
    ])
    assert rc == 0, 'Found code style errors / warnings'
