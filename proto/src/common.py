def substitute(template, replacements = []):

    for r in replacements:
        template = template.replace((r[0]), (r[1]))

    return template