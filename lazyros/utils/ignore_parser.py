import re
import os

class IgnoreParser:
    def __init__(self):
        self.ignore_file_path = os.path.expanduser("~/.config/lazyros/ignore.yaml")
        self._has_ignore_file = os.path.exists(self.ignore_file_path)
        self._ignore_patterns = self._load_ignore_patterns()

    def _load_ignore_patterns(self):
        patterns = {
            'node': [],
            'topic': [],
            'parameter': []
        }

        if not self._has_ignore_file:
            return patterns

        current_type = None
        try:
            with open(self.ignore_file_path, 'r') as f:
                for line in f:
                    line = line.strip()
                    if not line or line.startswith('#'):
                        continue
                    
                    if line.startswith('node'):
                        current_type = 'node'
                    elif line.startswith('topic'):
                        current_type = 'topic'
                    elif line.startswith('parameter'):
                        current_type = 'parameter'
                    elif current_type:
                        try:
                            patterns[current_type].append(self._glob_to_regex(line))
                        except Exception:
                            continue
        except (IOError, OSError, UnicodeDecodeError) as e:
            pass
        except Exception as e:
            pass

        return patterns

    def _glob_to_regex(self, glob_pattern):
        """Converts a glob pattern to a regex pattern."""

        regex = re.escape(glob_pattern)
        regex = regex.replace(r'\*', '.*')
        regex = regex.replace(r'\?', '.')
        regex = f"^{regex}$"
        return regex

    def should_ignore(self, item_name, item_type):
        """Checks if an item name should be ignored based on its type."""
        try:
            if item_type not in self._ignore_patterns:
                return False
            for pattern in self._ignore_patterns[item_type]:
                try:
                    if re.fullmatch(pattern, item_name):
                        return True
                except re.error:
                    continue
            return False
        except Exception:
            return False
